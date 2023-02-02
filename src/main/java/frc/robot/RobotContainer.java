package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AUTO;
import frc.robot.commands.AprilTagVission;
import frc.robot.commands.DriverControl;
import frc.robot.commands.TestSystems;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.AutoPathPlanner;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controller.DebouncedController;

public class RobotContainer {

  public static DriveTrain driveTrain = new DriveTrain();
  private DebouncedController controller = new DebouncedController(0);
  private SendableChooser<JanusRouteFactory> autoChooser = new SendableChooser<>();
  private SendableChooser<String> autoPathChooser = new SendableChooser<>(); //this one is a string as the Path Planner calls on a file instead of command

  public RobotContainer() {

    driveTrain.shuffleboard();
    driveTrain.setDefaultCommand(new DriverControl(driveTrain, controller.leftX, controller.leftY, controller.rightX));


  
    autoChooser.addOption("Janus X", AUTO.TEST_X_AUTO_PATH);
    autoChooser.addOption("Janus Y", AUTO.TEST_Y_AUTO_PATH);
    autoChooser.addOption("Janus XY", AUTO.TEST_XY_AUTO_PATH);
    autoChooser.addOption("Janus Theta", AUTO.TEST_THETA_AUTO_PATH);
    autoChooser.addOption("Janus Movement", AUTO.TEST_MOVEMENT_AUTO_PATH);
    autoChooser.addOption("Janus Command 1", AUTO.TEXT_COMMAND_1_AUTO_PATH);
    autoChooser.addOption("Janus Command 2", AUTO.TEXT_COMMAND_2_AUTO_PATH);
    autoChooser.addOption("Janus Command 3", AUTO.TEXT_COMMAND_3_AUTO_PATH);

    autoPathChooser.addOption("Test Path", "Test Path");
    autoPathChooser.addOption("Right Side 2 Game Piece", "Right Side 2 Game Piece");

    Shuffleboard.getTab("Auto").add(autoChooser);
    Shuffleboard.getTab("Auto Path Planner").add(autoPathChooser);
    // SmartDashboard.putData("-=TEST=- Auto Selector", autoChooser);
    configureBindings();
    //createSystemTestButtonBinding();
  }

  private void configureBindings() {
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    controller.a.whileTrue(new AprilTagVission(driveTrain, driveTrain.getLimelight(), Constants.AUTO.XY_PID_CONFIG, Constants.AUTO.THETA_PID_CONFIG));
    controller.b.whileTrue(new AutoAlign(driveTrain, driveTrain.getLimelight(), driveTrain.getBlinkin(), Constants.AUTO.ALIGN_XY_PID_CONFIG, Constants.AUTO.ALIGN_THETA_PID_CONFIG));
  }

  public void createSystemTestButtonBinding(){
    controller.a.toggleOnTrue(new TestSystems(controller.top, controller.bottom, controller.leftY, driveTrain));
  }

  public Command getAutonomousCommand(){
    // return new JanusAuto(driveTrain, autoChooser.getSelected().build());
    return AutoPathPlanner.runAuto(autoPathChooser.getSelected());
  }

}
