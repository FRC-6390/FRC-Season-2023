package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingMid extends SequentialCommandGroup {
  
  public GoingMid() {
    ElevatorCommand.isDone = true;
    addCommands(new InstantCommand(ElevatorCommand::reset), new ArmUp(Constants.ARM.SETPOINT_SCORE), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_MID));
  }
}
