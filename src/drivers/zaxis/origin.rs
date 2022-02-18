// SPDX-License-Identifier: GPL-3.0-or-later

// We need to define the place where Z=0.0mm. For this we have a sensor at the
// bottom that activates whenever the build plate reaches the bottom.

use crate::consts::zaxis::homing::*;

use super::prelude::*;
use crate::zaxis;

pub async fn calibrate_origin(mc: &mut zaxis::MotionControlAsync) {
    // max_speed here should be an argument.
    // We could be homing for build plate setup
    // Or we could be homing for the start of the print, and be submersed in resin
    // Different retract speed are needed.
    let max_speed = QUICK_HOMING_SPEED_MM_PER_SEC.mm();

    mc.controlled_stop();
    mc.wait(zaxis::Event::Idle).await;

    // The first thing is to get at the bottom of the zaxis.
    if !mc.bottom_sensor.active() {
        // We might be far away from the bottom, we want to go there quickly.
        mc.set_max_speed(max_speed);
        mc.set_target(Steps::MIN);
        mc.wait(zaxis::Event::BottomSensor(true)).await;

        mc.controlled_stop();
        mc.wait(zaxis::Event::Idle).await;
    }

    // We are now below the bottom z axis sensor transition point.
    // We go up a little and down again. It's actually faster this way.
    mc.set_max_speed((4.0*FINE_HOMING_SPEED_MM_PER_SEC).mm());
    mc.set_target(Steps::MAX);
    mc.wait(zaxis::Event::BottomSensor(false)).await;

    mc.set_max_speed(FINE_HOMING_SPEED_MM_PER_SEC.mm());
    mc.set_target(Steps::MIN);
    mc.wait(zaxis::Event::BottomSensor(true)).await;

    mc.set_origin(-BOTTOM_SENSOR_POSITION_MM.mm());

    mc.controlled_stop();
}
