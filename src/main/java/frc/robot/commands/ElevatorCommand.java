package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {

  //DUMMY numbers
  public double setpoint;
  private double rangeMax = setpoint + 1;
  private double rangeMin = setpoint - 1;
  public static PIDController pid;
  public static boolean isDone;
  
  public ElevatorCommand(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    isDone = false;
    pid = new PIDController(0.005, 0.0025, 0);
  }

  @Override
  public void execute() {   
    if((Elevator.getPosition() > rangeMin) && (Elevator.getPosition() < rangeMax)){
      isDone = true;
    } else{
      Elevator.set(pid.calculate(Elevator.getPosition(), setpoint));
    }
  }

  @Override
  public void end(boolean interrupted) {
    Elevator.set(0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
