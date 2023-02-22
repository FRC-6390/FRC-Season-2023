package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WashingMachine extends SubsystemBase {

  public static TalonFX washingMotor;
  public static TalonFX sideGripper;

  public WashingMachine() {}

  static{
    washingMotor = new TalonFX(Constants.WASHING_MACHINE.MOTOR_ID, "can");
    sideGripper = new TalonFX(Constants.GRIPPER.SIDE_GRIPPER, "can");
  }

  @Override
  public void periodic() {

  }

  public static void setWasher(double speed){
    washingMotor.set(ControlMode.PercentOutput, speed);
  }

  //Sets rollers
  public static void setGrippers(double speed){
    sideGripper.set(ControlMode.PercentOutput, -speed);
  }

  public static double getPos(){
    return washingMotor.getSelectedSensorPosition();
  }
  public static void reset(){
    washingMotor.setSelectedSensorPosition(0.0);
  }

  //Gets roller position
  public static double getGripperPosition(){
    return sideGripper.getSelectedSensorPosition();
  }

  //Resets roller encoder
  public static void resetGripperEncoder(){
    sideGripper.setSelectedSensorPosition(0);
  }
}

