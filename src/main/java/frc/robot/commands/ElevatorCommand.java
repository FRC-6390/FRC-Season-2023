package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {

  //DUMMY numbers
  public double setpoint;
  public static PIDController pid;
  public static boolean isDone;
  
  public ElevatorCommand(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    isDone = false;
    pid = new PIDController(0.0003, 0.0002, 0);
  }

  @Override
  public void execute() {   
    if(setpoint == Constants.ELEVATOR.SETPOINT_HIGH){
      if((Elevator.getPosition() > setpoint -50) && (Elevator.getPosition() < setpoint + 50)){
        isDone = true;
        System.out.println("Done");
      }
    }
    if((Elevator.getPosition() > setpoint -2) && (Elevator.getPosition() < setpoint +2)){
      isDone = true;
      System.out.println("Done");
    } else{
      Elevator.set(pid.calculate(Elevator.getPosition(), setpoint));
      System.out.println("Elevator Moving: " + Elevator.getPosition());
      if((Elevator.getLimitSwitch() == false) && (setpoint == 0)){
        Elevator.setPosition(0);
        isDone = true;
      }
    }
  }

  public static void reset(){
    isDone = true;
  }

  @Override
  public void end(boolean interrupted) {
    Elevator.set(0);
    System.out.println("Command Finished");
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
