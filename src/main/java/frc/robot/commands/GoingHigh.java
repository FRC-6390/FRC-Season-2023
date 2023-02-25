package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class GoingHigh extends SequentialCommandGroup {
  
  public GoingHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    CommandScheduler.getInstance().cancelAll();
    addCommands(new ArmUp(), new ElevatorCommand(Constants.ELEVATOR.SETPOINT_HIGH));
  }
}
