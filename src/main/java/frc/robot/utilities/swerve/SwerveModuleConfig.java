package frc.robot.utilities.swerve;

public record SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, double encoderOffset, String canbus) {
    
    public SwerveModuleConfig(int driveMotor, boolean driveMotorReversed, int rotationMotor, boolean rotationMotorReversed, int encoder, double encoderOffset, String canbus){
        this.driveMotor = driveMotor;
        this.driveMotorReversed = driveMotorReversed;
        this.rotationMotor = rotationMotor;
        this.rotationMotorReversed = rotationMotorReversed;
        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
        this.canbus = canbus;
    }
}
