package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRollers extends CommandBase {
  
  double speed;

  public IntakeRollers(double speed) {
    this.speed = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Intake.setRollers(speed);
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setRollers(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}