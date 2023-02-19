
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmUp extends CommandBase {

  //Dummy number
  public double setpoint = 0;
  public double prevError;
  public static PIDController pid;

  //boolean to decide when the command should end
  public boolean isDone;

  public ArmUp() {
  }

  @Override
  public void initialize() {
    isDone = false;
    Arm.currentPosition = false;
    Arm.armMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
