package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class GripperRollers extends CommandBase {
 
  public double speed;
  public boolean isDone = false;

  public GripperRollers(double speed) {
    this.speed = speed;
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(Arm.getRollersVoltage() > 4){
      Arm.setRollers(0);
      isDone = true;
    } else {
      Arm.setRollers(speed);
    }
  }

  
  @Override
  
  public void end(boolean interrupted) {
    //Stops the motors
    Arm.setRollers(0);
  }

  
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
