package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeMahdi;

public class IntakeOut extends CommandBase {
  
  double speed;
  public IntakeOut(double speed) {
    this.speed = speed;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() 
  {
    IntakeMahdi.setRollers(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

