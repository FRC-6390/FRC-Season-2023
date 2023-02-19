package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WashingMachine;

public class SpinWasher extends CommandBase {

  
  public double speed;

  //Sets up speed parameter
  public SpinWasher(double speed) {
    this.speed = speed;
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //Turns on the washer
    WashingMachine.setWasher(speed);

    //moving grippers
    WashingMachine.setGrippers(0.5);

    if(Math.abs(WashingMachine.getPos()) > 100000){
      WashingMachine.reset();
      speed *= -1;
    } 
  }

  @Override
  public void end(boolean interrupted) {
    //Turns off the washer and roller
    WashingMachine.setWasher(0);
    WashingMachine.setGrippers(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
