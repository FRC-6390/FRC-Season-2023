package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AUTO;
import frc.robot.commands.AprilTagVission;
import frc.robot.commands.DriverControl;
import frc.robot.commands.GripperRollers;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeRollers;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.SpinWasher;
import frc.robot.commands.TestSystems;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoPathPlanner;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controller.DebouncedController;
import frc.robot.utilities.controller.DebouncedJoystick;

public class RobotContainer {

  
  public static DriveTrain driveTrain = new DriveTrain();

  //controllers
  public static DebouncedController controller = new DebouncedController(0);
  private DebouncedJoystick joystick = new DebouncedJoystick(1);

  //smart dashboard auto selectors
  private SendableChooser<JanusRouteFactory> autoChooser = new SendableChooser<>();
  private SendableChooser<String> autoPathChooser = new SendableChooser<>(); 

  public static boolean intakeState = true;

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
    autoPathChooser.addOption("Left Side 2 Game Piece", "Left Side 2 Game Piece");
    autoPathChooser.addOption("Middle 1 Game Piece and Balance", "Middle 1 Game Piece and Balance");

    Shuffleboard.getTab("Auto").add(autoChooser);
    Shuffleboard.getTab("Auto Path Planner").add(autoPathChooser);

    configureBindings();
  }

  private void configureBindings() {

    //primary driver controls on XBOX Controller
    controller.start.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    controller.a.whileTrue(new AprilTagVission(driveTrain, driveTrain.getLimelight(), Constants.AUTO.XY_PID_CONFIG, Constants.AUTO.THETA_PID_CONFIG));
    controller.b.whileTrue(new AutoAlign(driveTrain, driveTrain.getLimelight(), driveTrain.getBlinkin(), Constants.AUTO.ALIGN_XY_PID_CONFIG, Constants.AUTO.ALIGN_THETA_PID_CONFIG));
    controller.y.onTrue(new AutoBalance(driveTrain));
    // controller.leftTrigger.whileTrue(new IntakeRollers(0.5));
    // controller.rightTrigger.whileTrue(new GripperRollers(0.5));

    controller.leftBumper.onTrue(new IntakeDown());
    controller.rightBumper.onTrue(new IntakeUp());

    

    //secondary driver controls on Logitech Controller
    joystick.seven.whileTrue(new SpinWasher(0.5));
    joystick.eight.whileTrue(new SpinWasher(-0.5));
    
  }

  public void createSystemTestButtonBinding(){
    controller.a.toggleOnTrue(new TestSystems(controller.top, controller.bottom, controller.leftY, driveTrain));
  }

  public Command getAutonomousCommand(){
    return AutoPathPlanner.runAuto(autoPathChooser.getSelected());
  }

}
