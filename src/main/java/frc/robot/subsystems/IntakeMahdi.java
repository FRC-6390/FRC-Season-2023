
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMahdi extends SubsystemBase {
  
  public static TalonFX intakeLift;
  public static TalonFX intakeRollerL;
  public static TalonFX intakeRollerR;
  public static CANCoder liftEnc;
  public static boolean pos;

  public IntakeMahdi() {}


  static
  {
    intakeLift = new TalonFX(Constants.INTAKE.ARM);
    intakeRollerL = new TalonFX(Constants.INTAKE.LEFT_MOTOR);
    intakeRollerR = new TalonFX(Constants.INTAKE.RIGHT_MOTOR);
    liftEnc = new CANCoder(Constants.INTAKE.POSITION_ENCODER);
  }

  @Override
  public void periodic() {
    
  }

  public static void setRollers(double speed)
  {
    intakeRollerL.set(ControlMode.PercentOutput, speed);
    intakeRollerR.set(ControlMode.PercentOutput, speed);
  }
  public static void setLift(double speed)
  {
    intakeLift.set(ControlMode.PercentOutput, speed);
  }
  public static double getPosition()
  {
    return liftEnc.getPosition()/4096d;
  }

  public static void resetEncoder()
  {
    liftEnc.setPosition(0);
  }

  
}
