package frc.robot.utilities.controlloop.motionprofile;

import frc.robot.utilities.controlloop.PIDConfig;

public record MotionProfileConfig(double maxSpeedMeters, double maxAccelerationMeters, PIDConfig PIDConfig) {

}
