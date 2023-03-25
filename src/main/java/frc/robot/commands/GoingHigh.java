package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingHigh extends SequentialCommandGroup {
  
  public GoingHigh() {
    ElevatorCommand.isDone = true;
    addCommands(new InstantCommand(ElevatorCommand::reset), new ArmUp(Constants.ARM.SETPOINT_SCORE, false), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_HIGH));
  }
}
