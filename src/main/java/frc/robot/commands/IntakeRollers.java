package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRollers extends CommandBase {

  private Intake intake;
  
  public IntakeRollers(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.rollerSpeed(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    intake.rollerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
