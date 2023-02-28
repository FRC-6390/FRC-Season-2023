package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingLow extends SequentialCommandGroup {
  
  public GoingLow() {
    ElevatorCommand.isDone = true;
    addCommands(new InstantCommand(ElevatorCommand::reset), new ArmUp(), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_LOW));
  }
}
