package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.WashingMachine;

public class SpinWasher extends CommandBase {

  public boolean isDone;
  public double washerSpeed, intakeSpeed;

  //Sets up speed parameter
  public SpinWasher(double washerSpeed, double intakeSpeed) {
    this.washerSpeed = washerSpeed;
    this.intakeSpeed = intakeSpeed;
  }
  
  @Override
  public void initialize() {
    isDone = false;
    WashingMachine.resetGripperEncoder();
  }

  @Override
  public void execute() {
    System.out.println(Intake.getRollerCurrent());

    if(Intake.getPosition() < -50){
      if(Intake.getRollerCurrent() > 50){
        new WaitCommand(1);
      } else{
        Intake.setRollers(intakeSpeed);
      }
    }

    //moving grippers
    WashingMachine.setGrippers(0.5);
    // if(WashingMachine.getGripperPosition() < -20000){

      //Turns on the washer after 1 second
      WashingMachine.setWasher(washerSpeed);

      if(Math.abs(WashingMachine.getPos()) > 100000){
        WashingMachine.reset();
        washerSpeed *= -1;
      } 
    // }

    // if(Arm.getRollersVoltage() < 10){
    //   Arm.setRoller(-0.5);
    // } else {
    //   isDone = true;
    // }

  }

  @Override
  public void end(boolean interrupted) {
    //Turns off the washer and roller
    Arm.setRoller(0);
    Intake.setRollers(0);
    WashingMachine.setWasher(0);
    WashingMachine.setGrippers(0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
