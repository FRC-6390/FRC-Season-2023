package frc.robot.utilities.swerve;

import frc.robot.utilities.controlloop.PIDConfig;

public record SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, int encoderOffset, String canbus) {
    
    public SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, int encoderOffset){
        this(driveMotor, driveMotorReversed, rotationMotor, rotationMotorReversed, encoder, encoderOffset, null);
    }
}
