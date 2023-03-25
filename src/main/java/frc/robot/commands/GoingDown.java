package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingDown extends SequentialCommandGroup {
  
  public GoingDown() {
    ElevatorCommand.isDone = true;
    System.out.println("___________________________" + ElevatorCommand.isDone);
    addCommands(new InstantCommand(ElevatorCommand::reset), new ArmUp(Constants.ARM.SETPOINT_SCORE, true), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_DOWN), new ArmDown());
  }
}
