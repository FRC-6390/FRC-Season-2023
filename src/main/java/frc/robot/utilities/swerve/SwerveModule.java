package frc.robot.utilities.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX rotationMotor;

    private CANCoder encoder;
    private PID pid;

    private double encoderOffset;
   // private REVMaglimitSwitch limitSwitch;

    public SwerveModule(SwerveModuleConfig config){

        if(config.canbus() != null){
            driveMotor = new TalonFX(config.driveMotor(), config.canbus());
            rotationMotor = new TalonFX(config.rotationMotor(), config.canbus());
            encoder = new CANCoder(config.encoder(), config.canbus());
        }else{
            driveMotor = new TalonFX(config.driveMotor());
            rotationMotor = new TalonFX(config.rotationMotor());
            encoder = new CANCoder(config.encoder());
        }

        driveMotor.setInverted(config.driveMotorReversed());
        rotationMotor.setInverted(config.rotationMotorReversed());

        driveMotor.configSelectedFeedbackCoefficient(SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS);
        rotationMotor.configSelectedFeedbackCoefficient(SWERVEMODULE.ROTATION_ENCODER_CONVERSION_RADIANS);
        pid = new PID(() -> getRotationMotorPosition(), null, SWERVEMODULE.ROTATION_PID);

        resetEncoders();
    }

    public double getDriveMotorVelocity(){
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getRotationMotorVelocity(){
        return rotationMotor.getSelectedSensorVelocity();
    }

    public double getDriveMotorPosition(){
        return driveMotor.getSelectedSensorPosition();
    }

    public double getRotationMotorPosition(){
        return rotationMotor.getSelectedSensorPosition();
    }

    public double getEncoderRadians(){
        return (encoder.getAbsolutePosition() * Math.PI/180d) + encoderOffset;
    }

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);
        rotationMotor.setSelectedSensorPosition(getEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getRotationMotorPosition()));
    }

    public SwerveModulePosition getPostion(){
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getRotationMotorPosition()));
    }

    public void setDriveMotor(double speed){
        driveMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setRotationMotor(double speed){
        rotationMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        rotationMotor.set(ControlMode.PercentOutput, pid.calculate(state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        rotationMotor.set(ControlMode.PercentOutput, 0);
    }

}
