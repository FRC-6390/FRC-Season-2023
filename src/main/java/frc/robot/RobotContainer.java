package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriverControl;
import frc.robot.commands.TestSystems;
import frc.robot.commands.auto.TestRoute1;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.controller.DebouncedController;

public class RobotContainer {

  private DriveTrain driveTrain = new DriveTrain();
  private DebouncedController controller = new DebouncedController(0);
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    driveTrain.setDefaultCommand(new DriverControl(driveTrain, controller.leftX, controller.leftY, controller.rightX));

    autoChooser.addOption("Test Route 1", new TestRoute1());

    SmartDashboard.putData("Auto Selector", autoChooser);;
    configureBindings();
  }

  private void configureBindings() {
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
  }

  public void createSystemTestButtonBinding(){
    controller.a.toggleOnTrue(new TestSystems(controller.top, controller.bottom, controller.leftY, driveTrain));
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

}
