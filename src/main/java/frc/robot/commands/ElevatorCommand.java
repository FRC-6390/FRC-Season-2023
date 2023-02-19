package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    pid = new PIDController(0.0008, 0.0, 0);
  }

  @Override
  public void execute() {   
    if((Elevator.getPosition() > setpoint -2) && (Elevator.getPosition() < setpoint +2)){
      isDone = true;
      System.out.println("Done");
    } else{
      // Elevator.set(pid.calculate(Elevator.getPosition(), setpoint));
      System.out.println("Elevator Moving: " + Elevator.getPosition());
    }
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
