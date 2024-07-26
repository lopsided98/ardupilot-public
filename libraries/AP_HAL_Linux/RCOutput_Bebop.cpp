#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include "RCOutput_Bebop.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"

#define BEBOP_BLDC_GPIO_0       (1 << 0)
#define BEBOP_BLDC_GPIO_1       (1 << 1)
#define BEBOP_BLDC_GPIO_2       (1 << 2)
#define BEBOP_BLDC_GPIO_3       (1 << 3)
#define BEBOP_BLDC_GPIO_POWER   (1 << 4)

#define BEBOP_BLDC_MIN_PERIOD_US 1100
#define BEBOP_BLDC_MAX_PERIOD_US 1900
#define BEBOP_BLDC_MIN_RPM 1000
/* the max rpm speed is different on Bebop 2 */
#define BEBOP_BLDC_MAX_RPM_1 11000
#define BEBOP_BLDC_MAX_RPM_2 12200
#define BEBOP_BLDC_MAX_RPM_DISCO 12500

/* Priority of the thread controlling the BLDC via i2c
 * set to 14, which is the same as the UART
 */
#define RCOUT_BEBOP_RTPRIO 14
/* Set timeout to 500ms */
#define BEBOP_BLDC_TIMEOUT_NS 500000000

namespace Linux
{

struct PACKED BebopBLDC_Info {
    uint8_t version_maj;
    uint8_t version_min;
    uint8_t type;
    uint8_t n_motors;
    uint16_t n_flights;
    uint16_t last_flight_time;
    uint32_t total_flight_time;
    uint8_t last_error;
};

enum class BebopBLDC_Sound: uint8_t {
    NONE = 0,
    SHORT_BEEP = 1,
    BOOT_BEEP = 2,
    BEBOP = 3,
};

// Bebop BLDC registers description
enum BebopBLDC_Command: uint8_t {
    SET_REF_SPEED = 0x02,
    GET_OBS_DATA = 0x20,
    START_PROP = 0x40,
    TOGGLE_GPIO = 0x4d,
    STOP_PROP = 0x60,
    CLEAR_ERROR = 0x80,
    PLAY_SOUND = 0x82,
    GET_INFO = 0xA0,
};

};

using namespace Linux;

extern const AP_HAL::HAL& hal;

RCOutput_Bebop::RCOutput_Bebop(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
    , _min_pwm(BEBOP_BLDC_MIN_PERIOD_US)
    , _max_pwm(BEBOP_BLDC_MAX_PERIOD_US)
    , _period_us{}
    , _request_period_us{}
    , _rpm{}
{
}

uint8_t RCOutput_Bebop::_checksum(uint8_t *data, size_t len)
{
    uint8_t checksum = data[0];

    for (size_t i = 1; i < len; i++) {
        checksum = checksum ^ data[i];
    }

    return checksum;
}

void RCOutput_Bebop::_start_prop()
{
    uint8_t data = BebopBLDC_Command::START_PROP;

    WITH_SEMAPHORE(_dev->get_semaphore());
    if (_dev->transfer(&data, sizeof(data), nullptr, 0)) {
        _state = State::STARTED;
    }
}

void RCOutput_Bebop::_set_ref_speed(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM])
{
    struct PACKED bldc_ref_speed_data {
        uint8_t     cmd;
        uint16_t    rpm[BEBOP_BLDC_MOTORS_NUM];
        uint8_t     enable_security;
        uint8_t     checksum;
    } data {};

    data.cmd = BebopBLDC_Command::SET_REF_SPEED;

    for (size_t i = 0; i < BEBOP_BLDC_MOTORS_NUM; i++) {
        data.rpm[i] = htobe16(rpm[i]);
    }

    data.enable_security = 0;
    data.checksum = _checksum(reinterpret_cast<uint8_t*>(&data), sizeof(data) - 1);

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer(reinterpret_cast<uint8_t*>(&data), sizeof(data), nullptr, 0);
}

bool RCOutput_Bebop::_get_info(BebopBLDC_Info &info)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!_dev->read_registers(BebopBLDC_Command::GET_INFO, reinterpret_cast<uint8_t*>(&info), sizeof(info))) {
        return false;
    }
    return true;
}

int RCOutput_Bebop::read_obs_data(BebopBLDC_ObsData &obs)
{
    // The structure returned is different on the Disco from the Bebop
    struct PACKED {
        uint16_t    rpm[BEBOP_BLDC_MOTORS_NUM];
        uint16_t    batt_mv;
        uint8_t     status;
        uint8_t     error;
        uint8_t     motors_err;
        uint8_t     temp;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
        // bit 0 indicates an overcurrent on the RC receiver port when high
        // bits #1-#6 indicate an overcurrent on the #1-#6 PPM servos
        uint8_t     overcurrent;
#endif
        uint8_t     checksum;
    } data {};

    {
        WITH_SEMAPHORE(_dev->get_semaphore());
        if (!_dev->read_registers(BebopBLDC_Command::GET_OBS_DATA, reinterpret_cast<uint8_t*>(&data), sizeof(data))) {
            return -EIO;
        }
    }

    if (data.checksum != _checksum(reinterpret_cast<uint8_t*>(&data), sizeof(data) - 1)) {
        return -EBUSY;
    }

    /* fill obs class */
    obs = {};
    for (uint8_t i = 0; i < _n_motors; i++) {
        /* extract 'rpm saturation bit' */
        obs.rpm_saturated[i] = (data.rpm[i] & (1 << 7)) ? 1 : 0;
        /* clear 'rpm saturation bit' */
        data.rpm[i] &= static_cast<uint16_t>(~(1 << 7));
        obs.rpm[i] = be16toh(data.rpm[i]);
        if (obs.rpm[i] == 0) {
            obs.rpm_saturated[i] = 0;
        }
#if 0
        printf("rpm %u %u %u %u status 0x%02x temp %u\n",
               obs.rpm[i], _rpm[0], _period_us[0], _period_us_to_rpm(_period_us[0]),
               data.status, data.temp);
#endif
    }

    // sync our state from status. This makes us more robust to i2c errors
    BebopBLDC_Status status = static_cast<BebopBLDC_Status>(data.status & 0x0F);
    switch (status) {
    case BebopBLDC_Status::IDLE:
    case BebopBLDC_Status::STOPPING:
        _state = State::STOPPED;
        break;
    case BebopBLDC_Status::RAMPING:
    case BebopBLDC_Status::SPINNING_2:
        _state = State::STARTED;
        break;
    default:
        break;
    }

    obs.batt_mv = be16toh(data.batt_mv);
    obs.status = status;
    obs.error = static_cast<BebopBLDC_Error>(data.error);
    obs.motors_err = data.motors_err;
    obs.temperature = data.temp;

    return 0;
}

void RCOutput_Bebop::_toggle_gpio(uint8_t mask)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->write_register(BebopBLDC_Command::TOGGLE_GPIO, mask);
}

void RCOutput_Bebop::_stop_prop()
{
    uint8_t data = BebopBLDC_Command::STOP_PROP;

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer(&data, sizeof(data), nullptr, 0);
}

void RCOutput_Bebop::_clear_error()
{
    uint8_t data = BebopBLDC_Command::CLEAR_ERROR;

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer(&data, sizeof(data), nullptr, 0);
}

void RCOutput_Bebop::_play_sound(BebopBLDC_Sound sound)
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->write_register(BebopBLDC_Command::PLAY_SOUND, static_cast<uint8_t>(sound));
}

/*
 * pwm is the pwm power used for the note.
 *  It has to be >= 3, otherwise it refers to a predefined song
 * (see _play_sound function)
 * period is in us and duration in ms.
 */
void RCOutput_Bebop::play_note(uint8_t pwm,
                               uint16_t period_us,
                               uint16_t duration_ms)
{
    struct PACKED {
        uint8_t cmd;
        uint8_t pwm;
        be16_t period;
        be16_t duration;
    } msg;

    if (pwm < 3) {
        return;
    }

    msg.cmd = BebopBLDC_Command::PLAY_SOUND;
    msg.pwm = pwm;
    msg.period = htobe16(period_us);
    msg.duration = htobe16(duration_ms);

    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->transfer(reinterpret_cast<uint8_t*>(&msg), sizeof(msg), nullptr, 0);
}

uint16_t RCOutput_Bebop::_period_us_to_rpm(uint16_t period_us)
{
    period_us = constrain_int16(period_us, _min_pwm, _max_pwm);
    float period_us_fl = period_us;
    float rpm_fl = (period_us_fl - _min_pwm)/(_max_pwm - _min_pwm) *
                   (_max_rpm - BEBOP_BLDC_MIN_RPM) + BEBOP_BLDC_MIN_RPM;

    return static_cast<uint16_t>(rpm_fl);
}

void RCOutput_Bebop::init()
{
    int ret=0;
    struct sched_param param = { .sched_priority = RCOUT_BEBOP_RTPRIO };
    pthread_attr_t attr;
    pthread_condattr_t cond_attr;

    /* Initialize thread, cond, and mutex */
    ret = pthread_mutex_init(&_mutex, nullptr);
    if (ret != 0) {
        AP_HAL::panic("RCOutput_Bebop: failed to init mutex: %s", strerror(errno));
        return;
    }

    pthread_mutex_lock(&_mutex);

    pthread_condattr_init(&cond_attr);
    pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
    ret = pthread_cond_init(&_cond, &cond_attr);
    pthread_condattr_destroy(&cond_attr);
    if (ret != 0) {
        AP_HAL::panic("RCOutput_Bebop: failed to init cond: %s", strerror(errno));
        goto exit;
    }

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        AP_HAL::panic("RCOutput_Bebop: failed to init attr: %s", strerror(errno));
        goto exit;
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _control_thread, this);
    if (ret != 0) {
        AP_HAL::panic("RCOutput_Bebop: failed to create thread: %s", strerror(errno));
        goto exit;
    }

    _clear_error();

    /* Set an initial dummy frequency */
    _frequency = 50;

    // enable servo power (also receiver power)
    _toggle_gpio(BEBOP_BLDC_GPIO_2 | BEBOP_BLDC_GPIO_POWER);

exit:
    pthread_mutex_unlock(&_mutex);
    return;
}

void RCOutput_Bebop::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _frequency = freq_hz;
}

uint16_t RCOutput_Bebop::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_Bebop::enable_ch(uint8_t ch)
{
}

void RCOutput_Bebop::disable_ch(uint8_t ch)
{
    if (ch < _n_motors) {
        _stop_prop();
    }
}

void RCOutput_Bebop::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= BEBOP_BLDC_MOTORS_NUM) {
        return;
    }

    _request_period_us[ch] = period_us;

    if (!_corking) {
        push();
    }
}

void RCOutput_Bebop::cork()
{
    _corking = true;
}

void RCOutput_Bebop::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;
    pthread_mutex_lock(&_mutex);
    memcpy(_period_us, _request_period_us, sizeof(_period_us));
    pthread_cond_signal(&_cond);
    pthread_mutex_unlock(&_mutex);
    memset(_request_period_us, 0, sizeof(_request_period_us));
}

uint16_t RCOutput_Bebop::read(uint8_t ch)
{
    if (ch < BEBOP_BLDC_MOTORS_NUM) {
        return _period_us[ch];
    } else {
        return 0;
    }
}

void RCOutput_Bebop::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}

/* Separate thread to handle the Bebop motors controller */
void* RCOutput_Bebop::_control_thread(void *arg)
{
    RCOutput_Bebop* rcout = static_cast<RCOutput_Bebop*>(arg);

    rcout->_run_rcout();
    return nullptr;
}

void RCOutput_Bebop::_run_rcout()
{
    uint16_t current_period_us[BEBOP_BLDC_MOTORS_NUM] {};
    uint8_t i;
    int ret;
    struct timespec ts;
    BebopBLDC_Info info;
    BebopBLDC_Motor bebop_bldc_channels[BEBOP_BLDC_MOTORS_NUM] {};
    int hw_version;

    if (!_get_info(info)) {
        AP_HAL::panic("RCOutput_Bebop: failed to get BLDC info");
    }

    // remember _n_motors for read_obs_data()
    _n_motors = info.n_motors;

#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_DISCO
    BebopBLDC_Motor bebop_bldc_right_front, bebop_bldc_left_front,
                    bebop_bldc_left_back, bebop_bldc_right_back;
    /* Set motor order depending on BLDC version.On bebop 1 with version 1
     * keep current order. The order changes from version 2 on bebop 1 and
     * remains the same as this for bebop 2
     */
    if (info.version_maj == 1) {
        bebop_bldc_right_front = BebopBLDC_Motor::MOTOR_1;
        bebop_bldc_left_front  = BebopBLDC_Motor::MOTOR_2;
        bebop_bldc_left_back   = BebopBLDC_Motor::MOTOR_3;
        bebop_bldc_right_back  = BebopBLDC_Motor::MOTOR_4;
    } else {
        bebop_bldc_right_front = BebopBLDC_Motor::MOTOR_2;
        bebop_bldc_left_front  = BebopBLDC_Motor::MOTOR_1;
        bebop_bldc_left_back   = BebopBLDC_Motor::MOTOR_4;
        bebop_bldc_right_back  = BebopBLDC_Motor::MOTOR_3;
    }

    bebop_bldc_channels[0] = bebop_bldc_right_front;
    bebop_bldc_channels[1] = bebop_bldc_left_back;
    bebop_bldc_channels[2] = bebop_bldc_left_front;
    bebop_bldc_channels[3] = bebop_bldc_right_back;
#endif

    hw_version = Util::from(hal.util)->get_hw_arm32();
    if (hw_version == UTIL_HARDWARE_BEBOP) {
        _max_rpm = BEBOP_BLDC_MAX_RPM_1;
    } else if (hw_version == UTIL_HARDWARE_BEBOP2) {
        _max_rpm = BEBOP_BLDC_MAX_RPM_2;
    } else if (hw_version == UTIL_HARDWARE_DISCO) {
        _max_rpm = BEBOP_BLDC_MAX_RPM_DISCO;
    } else if (hw_version < 0) {
        AP_HAL::panic("RCOutput_Bebop: failed to get hw version %s", strerror(hw_version));
    } else {
        AP_HAL::panic("RCOutput_Bebop: unsupported hw version %d", hw_version);
    }
    printf("Bebop: vers %u/%u type %u nmotors %u n_flights %u last_flight_time %u total_flight_time %u maxrpm %u\n",
           info.version_maj, info.version_min, info.type, info.n_motors,
		   be16toh(info.n_flights), be16toh(info.last_flight_time),
		   be32toh(info.total_flight_time), _max_rpm);

    while (true) {
        pthread_mutex_lock(&_mutex);
        ret = clock_gettime(CLOCK_MONOTONIC, &ts);
        if (ret != 0) {
            pthread_mutex_unlock(&_mutex);
            continue;
        }

        if (ts.tv_nsec > (1000000000 - BEBOP_BLDC_TIMEOUT_NS)) {
            ts.tv_sec += 1;
            ts.tv_nsec = ts.tv_nsec + BEBOP_BLDC_TIMEOUT_NS - 1000000000;
        } else {
            ts.tv_nsec += BEBOP_BLDC_TIMEOUT_NS;
        }

        ret = 0;
        while ((memcmp(_period_us, current_period_us, sizeof(_period_us)) == 0) && (ret == 0)) {
            ret = pthread_cond_timedwait(&_cond, &_mutex, &ts);
        }

        memcpy(current_period_us, _period_us, sizeof(_period_us));
        pthread_mutex_unlock(&_mutex);

        /* start propellers if the speed of the 4 motors is >= min speed
         * min speed set to min_pwm + 50*/
        for (i = 0; i < _n_motors; i++) {
            if (current_period_us[i] <= _min_pwm + 50) {
                break;
            }
            _rpm[static_cast<size_t>(bebop_bldc_channels[i])] = _period_us_to_rpm(current_period_us[i]);
        }

        if (i < _n_motors) {
            /* one motor pwm value is at minimum (or under)
             * if the motors are started, stop them*/
            if (_state == State::STARTED) {
                _stop_prop();
                _clear_error();
            }
        } else {
            /* all the motor pwm values are higher than minimum
             * if the bldc is stopped, start it*/
            if (_state == State::STOPPED) {
                _start_prop();
            }
            _set_ref_speed(_rpm);
        }
    }
}

#endif
