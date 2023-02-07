package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WashingMachine;

public class SpinWasher extends CommandBase {

  public double speed;
  public SpinWasher(double speed) {
    this.speed = speed;
  }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    WashingMachine.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    WashingMachine.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
