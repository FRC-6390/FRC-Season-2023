
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  public static TalonFX intakeLift;
  public static TalonFX intakeRoller;
  public static CANCoder liftEncoder;
  public static DigitalInput intakeLimitSwitch;

  public Intake() {

  }

  static{
    intakeLift = new TalonFX(Constants.INTAKE.ARM, "can");
    intakeRoller = new TalonFX(Constants.INTAKE.INTAKE_MOTOR, "can");
    liftEncoder = new CANCoder(Constants.INTAKE.POSITION_ENCODER, "can");
    intakeLimitSwitch = new DigitalInput(Constants.INTAKE.LIMIT_SWITCH); 
  }

  @Override
  public void periodic() {
    
  }

  //Sets the intake rollers
  public static void setRollers(double speed){
    intakeRoller.set(ControlMode.PercentOutput, speed);
  }

  //Sets the lift to a certain speed
  public static void setLift(double speed){
    intakeLift.set(ControlMode.PercentOutput, speed);
  }

  //Gets lift position
  public static double getPosition(){
    // return liftEncoder.getPosition();
    return intakeLift.getSelectedSensorPosition();
  }


  //Gets roller position
  public static double getRollerPosition(){
    return intakeRoller.getSelectedSensorPosition();
  }

  public static double getRollerCurrent(){
    return intakeRoller.getSupplyCurrent();
  }

  //Resets lift encoder
  public static void resetEncoder(){
    // liftEncoder.setPosition(0);
    intakeLift.setSelectedSensorPosition(0);
  }

  //Resets roller encoder
  public static void resetRollerEncoder(){
    intakeRoller.setSelectedSensorPosition(0);
  }

  //Get value of the intake lift limit switch
  public static boolean getLimitSwitch(){
    //false for triggered, otherwise true
    return intakeLimitSwitch.get();
  }
}
