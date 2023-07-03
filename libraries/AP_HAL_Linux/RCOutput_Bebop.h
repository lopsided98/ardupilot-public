#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/I2CDevice.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#include <atomic>
#include <pthread.h>

namespace Linux
{

enum class BebopBLDC_Motor: uint8_t {
    MOTOR_1,
#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_DISCO
    MOTOR_2,
    MOTOR_3,
    MOTOR_4,
#endif
    MOTORS_NUM,
};

const uint8_t BEBOP_BLDC_MOTORS_NUM = static_cast<uint8_t>(BebopBLDC_Motor::MOTORS_NUM);

struct BebopBLDC_Info;
enum class BebopBLDC_Sound: uint8_t;

// Values of bottom nibble of the obs data status byte
enum class BebopBLDC_Status: uint8_t {
    INIT = 0,
    IDLE = 1,
    RAMPING = 2,
    SPINNING_1 = 3,
    SPINNING_2 = 4,
    STOPPING = 5,
    CRITICAL = 6,
};

// Description of the BLDC errno
enum class BebopBLDC_Error : uint8_t {
    NONE = 0,
    EEPROM = 1,
    MOTOR_STALLED = 2,
    PROP_SECU = 3,
    COM_LOST = 4,
    BATT_LEVEL = 9,
    LIPO = 10,
    MOTOR_HW = 11
};

struct BebopBLDC_ObsData {
    uint16_t rpm[BEBOP_BLDC_MOTORS_NUM];
    uint8_t rpm_saturated[BEBOP_BLDC_MOTORS_NUM];
    uint16_t batt_mv;
    BebopBLDC_Status status;
    BebopBLDC_Error error;
    uint8_t motors_err;
    uint8_t temperature;
};

class RCOutput_Bebop : public AP_HAL::RCOutput, public AP_ESC_Telem_Backend
{
public:
    enum class State {
        STARTED,
        STOPPED,
        ERROR,
    };

    RCOutput_Bebop(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    static RCOutput_Bebop *from(AP_HAL::RCOutput *rcout)
    {
        return static_cast<RCOutput_Bebop*>(rcout);
    }

    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     play_note(uint8_t pwm, uint16_t period_us, uint16_t duration_ms);
    void     update();
    const BebopBLDC_ObsData& get_obs_data() const { return _obs; };

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint16_t _request_period_us[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _period_us[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _rpm[BEBOP_BLDC_MOTORS_NUM];
    BebopBLDC_Motor _channel_to_index[BEBOP_BLDC_MOTORS_NUM];
    uint8_t  _index_to_channel[BEBOP_BLDC_MOTORS_NUM];
    uint16_t _frequency;
    uint16_t _min_pwm;
    uint16_t _max_pwm;
    uint8_t  _n_motors{4};
    std::atomic<State> _state{State::STOPPED};
    bool     _corking{false};
    uint16_t _max_rpm;
    BebopBLDC_ObsData _obs;

    uint8_t _checksum(uint8_t *data, unsigned int len);
    void _start_prop();
    void _toggle_gpio(uint8_t mask);
    void _set_ref_speed(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM]);
    bool _get_info(BebopBLDC_Info &info);
    bool _read_obs_data(BebopBLDC_ObsData &obs);
    void _stop_prop();
    void _clear_error();
    void _play_sound(BebopBLDC_Sound sound);
    uint16_t _period_us_to_rpm(uint16_t period_us);

    /* thread related members */
    pthread_t _thread;
    pthread_mutex_t _mutex;
    pthread_cond_t _cond;
    void _run_rcout();
    static void *_control_thread(void *arg);
};

}
