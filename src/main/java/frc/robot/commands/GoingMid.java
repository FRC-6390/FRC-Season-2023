package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingMid extends SequentialCommandGroup {
  
  public GoingMid() {
    addCommands(new ArmUp(), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_MID));
  }
}
