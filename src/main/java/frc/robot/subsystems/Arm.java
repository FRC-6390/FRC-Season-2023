package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  
  public static TalonFX armMotor = new TalonFX(Constants.ARM.ARM_MOTOR, "can");
  public static CANCoder armEncoder = new CANCoder(Constants.ARM.ARM_ENCODER, "can");
  public static TalonFX outputRoller = new TalonFX(Constants.ARM.OUTPUT_ROLLER, "can");
  public static boolean currentPosition;

  public Arm() {

  }

  //Gets lift position
  public static double getPosition(){
    return armEncoder.getPosition();
  }

  //Gets lift position
  public static double getOutputRollerPosition(){
    return outputRoller.getSelectedSensorPosition();
  }

  public static void resetOutputRollerPosition(){
    outputRoller.setSelectedSensorPosition(0);
  }


  //Resets lift encoder
  public static void resetEncoder(){
    armEncoder.setPosition(0);
  }


  //Sets rollers
  public static void setRoller(double speed) {
    outputRoller.set(ControlMode.PercentOutput, speed);
  }

  public static double getRollersVoltage() {
    return outputRoller.getMotorOutputVoltage();
  }

  //Sets Arm Lift
  public static void setLift(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

}
