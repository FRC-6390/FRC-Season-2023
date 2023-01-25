package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeControl extends CommandBase {

  private Intake intake;
  
  public IntakeControl(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setIntake(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
