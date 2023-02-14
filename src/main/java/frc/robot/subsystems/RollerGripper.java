package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerGripper extends SubsystemBase {
  public RollerGripper() {}

  public static TalonFX leftRoller, rightRoller;
 

  static{
    leftRoller = new TalonFX(Constants.ROLLER_GRIPPER.LEFT_MOTOR);
    rightRoller = new TalonFX(Constants.ROLLER_GRIPPER.RIGHT_MOTOR);
    
  }

  @Override
  public void periodic() {
    
  }

  //Sets rollers
  public static void set(double speed)
  {
    leftRoller.set(ControlMode.PercentOutput, speed);
    rightRoller.set(ControlMode.PercentOutput, speed);
  }
}
