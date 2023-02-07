
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  public static TalonFX intakeLift;
  public static TalonFX intakeRollerLeft;
  public static TalonFX intakeRollerRight;
  public static CANCoder liftEncoder;
  public static DigitalInput intakeLimitSwitch;

  //false when up, true when intake is down
  public static boolean currentPosition;

  public Intake() {

  }

  static{
    currentPosition = false;
    intakeLift = new TalonFX(Constants.INTAKE.ARM);
    intakeRollerLeft = new TalonFX(Constants.INTAKE.LEFT_MOTOR);
    intakeRollerRight = new TalonFX(Constants.INTAKE.RIGHT_MOTOR);
    liftEncoder = new CANCoder(Constants.INTAKE.POSITION_ENCODER);
    intakeLimitSwitch = new DigitalInput(Constants.INTAKE.LIMIT_SWITCH);
  }

  @Override
  public void periodic() {
    
  }

  public static void setRollers(double speed){
    intakeRollerLeft.set(ControlMode.PercentOutput, speed);
    intakeRollerRight.set(ControlMode.PercentOutput, -speed);
  }

  public static void setLift(double speed){
    intakeLift.set(ControlMode.PercentOutput, speed);
  }

  public static double getPosition(){
    return liftEncoder.getPosition()/4096d;
  }

  public static void resetEncoder(){
    liftEncoder.setPosition(0);
  }

  public static boolean getLimitSwitch(){
    //false for triggered, otherwise true
    return intakeLimitSwitch.get();
  }
}
