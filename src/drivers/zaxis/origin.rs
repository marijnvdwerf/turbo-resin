// SPDX-License-Identifier: GPL-3.0-or-later

// We need to define the place where Z=0.0mm. For this we have a sensor at the
// bottom that activates whenever the build plate reaches the bottom.

use crate::consts::zaxis::origin_calibration::*;

use super::prelude::*;
use crate::zaxis;
use crate::debug;

pub async fn calibrate_origin(mc: &mut zaxis::MotionControlAsync) {
    // max_speed here should be an argument.
    // We could be homing for build plate setup
    // Or we could be homing for the start of the print, and be submersed in resin
    // Different retract speed are needed.
    let max_speed = PHASE1_HOMING_SPEED_MM_PER_SEC.mm();

    mc.stop();
    mc.wait(zaxis::Event::Idle).await;

    // The first thing is to get at the bottom of the zaxis.
    if !mc.bottom_sensor.active() {
        // We might be far away from the bottom, we want to go there quickly.
        mc.set_max_speed(max_speed);
        mc.set_target(Steps::MIN);
        mc.wait(zaxis::Event::BottomSensor(true)).await;

        mc.stop();
        mc.wait(zaxis::Event::Idle).await;
    }
    // We are now below the bottom z axis sensor.

    // We go up a little and down again. It's actually faster this way.
    mc.set_max_speed(PHASE2_HOMING_SPEED_MM_PER_SEC.mm());
    mc.set_target(Steps::MAX);
    mc.wait(zaxis::Event::BottomSensor(false)).await;
    // Go slighly higher to avoid noisy sensor problems. I have not verified
    // that it was a problem, but who knows. We are willing to pay 0.5s of
    // traveling.
    mc.set_target_relative((PHASE3_HOMING_SPEED_MM_PER_SEC/2.0).mm());
    mc.wait(zaxis::Event::Idle).await;

    mc.set_max_speed(PHASE3_HOMING_SPEED_MM_PER_SEC.mm());
    mc.set_target(Steps::MIN);
    mc.wait(zaxis::Event::BottomSensor(true)).await;

    mc.set_origin(-BOTTOM_SENSOR_POSITION_MM.mm());

    mc.stop();
    mc.wait(zaxis::Event::Idle).await;
}
