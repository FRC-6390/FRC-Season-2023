package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerGripper;

public class GripperCommand extends CommandBase {
 
  public double speed;
  
  public GripperCommand(double speed) {
    this.speed = speed;
   
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() 
  {
    RollerGripper.set(speed);
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
