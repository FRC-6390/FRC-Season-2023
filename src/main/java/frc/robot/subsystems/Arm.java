package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  public static boolean currentPosition;

  public Arm() {

  }

  public static TalonFX leftRoller, rightRoller, armMotor;
  public static CANCoder armEncoder;

  static{
    currentPosition = false;
    armMotor = new TalonFX(Constants.ARM.ARM_MOTOR);
    armEncoder = new CANCoder(Constants.ARM.ARM_ENCODER);
    leftRoller = new TalonFX(Constants.ARM.LEFT_ROLLER_GRIPPER);
    rightRoller = new TalonFX(Constants.ARM.LEFT_ROLLER_GRIPPER);
  }

  @Override
  public void periodic() {
    
  }

  //Sets rollers
  public static void setRollers(double speed) {
    leftRoller.set(ControlMode.PercentOutput, speed);
    rightRoller.set(ControlMode.PercentOutput, -speed);
  }

  public static double getRollersVoltage() {
    double volts = leftRoller.getMotorOutputVoltage() + rightRoller.getMotorOutputVoltage() / 2;
    return volts;
  }

  //Sets Arm Lift
  public static void setLift(double speed) {
    leftRoller.set(ControlMode.PercentOutput, speed);
  }

  public static double getPosition(){
    return armEncoder.getPosition()/4096d;
  }
}
