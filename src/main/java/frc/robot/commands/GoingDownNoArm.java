package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingDownNoArm extends SequentialCommandGroup {
  
  public GoingDownNoArm() {
    ElevatorCommand.isDone = true;
    System.out.println("___________________________" + ElevatorCommand.isDone);
    addCommands(new InstantCommand(ElevatorCommand::reset), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_DOWN), new ArmDown());
  }
}
