package frc.robot.utilities.swerve;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX rotationMotor;

    private CANCoder encoder;
    private PID pid;
    private GenericEntry offsetEntry;
    private SwerveModuleState state = new SwerveModuleState();

    private double encoderOffset;
   // private REVMaglimitSwitch limitSwitch;
    private static int instances = 0;

    public SwerveModule(SwerveModuleConfig config){
        this(config, null);
    }

    public SwerveModule(SwerveModuleConfig config, ShuffleboardTab tab){
       
        if(config.canbus() != null){
            driveMotor = new TalonFX(config.driveMotor(), config.canbus());
            rotationMotor = new TalonFX(config.rotationMotor(), config.canbus());
            encoder = new CANCoder(config.encoder(), config.canbus());
        }else{
            driveMotor = new TalonFX(config.driveMotor());
            rotationMotor = new TalonFX(config.rotationMotor());
            encoder = new CANCoder(config.encoder());
        }
        encoderOffset = config.encoderOffset();
        driveMotor.setInverted(config.driveMotorReversed());
        rotationMotor.setInverted(config.rotationMotorReversed());

        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        pid = new PID(SWERVEMODULE.ROTATION_PID).setMeasurement(() -> getRotationMotorPosition());
        if(tab != null){
            ShuffleboardLayout layout = tab.getLayout("Swerve Module "+instances, BuiltInLayouts.kList).withSize(2, 6);
            layout.add(pid);
            offsetEntry = layout.add("Offset "+ instances, getEncoderOffset()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -Math.PI, "max", Math.PI)).getEntry();
            layout.addDouble("Angle "+instances, () -> getPostion().angle.getDegrees());
            layout.addDouble("Absolute "+instances, () -> getAbsolutePosition());
        }
        instances++;

        resetEncoders();
        unlock();
    }

    public double getDriveMotorVelocity(){
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity() / 2048d /2*Math.PI * SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS;
    }
    
    public double getDriveMotorPosition(){
        return driveMotor.getSensorCollection().getIntegratedSensorPosition() / 2048d /2*Math.PI * SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS;
    }

    public double getRotationMotorPosition(){
        return getEncoderRadians();
    }

    public double getAbsolutePosition(){
        return encoder.getAbsolutePosition()* Math.PI/180d;
    }

    public double getEncoderOffset(){
        return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    public double getEncoderRadians(){
        if(offsetEntry != null) encoderOffset = offsetEntry.getDouble(0.0);

        return (encoder.getAbsolutePosition() * Math.PI/180d) - encoderOffset;
    }

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);
        rotationMotor.setSelectedSensorPosition(getEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getEncoderRadians()));
    }

    public SwerveModulePosition getPostion(){
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getEncoderRadians()));
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
        this.state = state;

    }

    public void stop(){
        driveMotor.set(ControlMode.PercentOutput, 0);
        rotationMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setToAngle(double angle){
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(angle));
        //setDesiredState(state);
        rotationMotor.set(ControlMode.PercentOutput, pid.calculate(state.angle.getRadians()));
    }

    public void lock(){
        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void unlock(){
        driveMotor.setNeutralMode(NeutralMode.Coast);
        rotationMotor.setNeutralMode(NeutralMode.Coast);
    }

}
