package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingLow extends SequentialCommandGroup {
  
  public GoingLow() {
    CommandScheduler.getInstance().cancelAll();
    addCommands(new ArmUp(), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_LOW), new ArmDown());
  }
}
