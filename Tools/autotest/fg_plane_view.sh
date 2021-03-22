#!/bin/sh

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native-fdm=socket,in,60,,5503,udp \
    --fdm=external \
    --aircraft=Rascal110-JSBSim \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=YKRY \
    --geometry=650x550 \
    --bpp=32 \
    --prop:/sim/rendering/multi-sample-buffers=1 \
    --prop:/sim/rendering/multi-samples=4 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*
