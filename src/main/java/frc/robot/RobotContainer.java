package frc.robot;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AUTO;
import frc.robot.commands.DriverControl;
import frc.robot.commands.TestSystems;
import frc.robot.commands.auto.JanusAuto;
import frc.robot.commands.auto.TestRoute1;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controller.DebouncedController;

public class RobotContainer {

  private DriveTrain driveTrain = new DriveTrain();
  private DebouncedController controller = new DebouncedController(0);
  private SendableChooser<JanusRouteFactory> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    driveTrain.setDefaultCommand(new DriverControl(driveTrain, controller.leftX, controller.leftY, controller.rightX));

    autoChooser.addOption("Janus X", AUTO.TEST_X_AUTO_PATH);
    autoChooser.addOption("Janus Y", AUTO.TEST_Y_AUTO_PATH);
    autoChooser.addOption("Janus XY", AUTO.TEST_XY_AUTO_PATH);
    autoChooser.addOption("Janus Theta", AUTO.TEST_THETA_AUTO_PATH);
    autoChooser.addOption("Janus Movement", AUTO.TEST_MOVEMENT_AUTO_PATH);
    autoChooser.addOption("Janus Command 1", AUTO.TEXT_COMMAND_1_AUTO_PATH);
    autoChooser.addOption("Janus Command 2", AUTO.TEXT_COMMAND_2_AUTO_PATH);
    autoChooser.addOption("Janus Command 3", AUTO.TEXT_COMMAND_3_AUTO_PATH);


    SmartDashboard.putData("-=TEST=- Auto Selector", autoChooser);;
    //configureBindings();
    createSystemTestButtonBinding();
  }

  private void configureBindings() {
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    // controller.a.whileTrue(new InstantCommand(() -> {
    //   System.out.println("a");
    // }));
  }

  public void createSystemTestButtonBinding(){
    controller.a.toggleOnTrue(new TestSystems(controller.top, controller.bottom, controller.leftY, driveTrain));
  }

  public Command getAutonomousCommand(){
    return new JanusAuto(driveTrain, autoChooser.getSelected().build());
  }

}
