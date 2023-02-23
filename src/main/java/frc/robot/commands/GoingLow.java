package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingLow extends SequentialCommandGroup {
  
  public GoingLow() {
    addCommands(new ArmDown(), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_LOW));
  }
}
