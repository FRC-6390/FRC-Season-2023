package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AUTO;
import frc.robot.commands.AprilTagVission;
import frc.robot.commands.DriverControl;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.SpinLeft;
import frc.robot.commands.SpinOff;
import frc.robot.commands.SpinRight;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.TestSystems;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.AutoPathPlanner;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.WashingMachineMahdi;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controller.DebouncedButton;
import frc.robot.utilities.controller.DebouncedController;

public class RobotContainer {

  public static DriveTrain driveTrain = new DriveTrain();
  private DebouncedController controller = new DebouncedController(0);
  private SendableChooser<JanusRouteFactory> autoChooser = new SendableChooser<>();
  private Joystick joystick = new Joystick(0);
  private DebouncedButton button1 = new DebouncedButton(joystick, 1);
  private DebouncedButton button2 = new DebouncedButton(joystick, 2);
  private DebouncedButton button3 = new DebouncedButton(joystick, 3);
  private DebouncedButton button4 = new DebouncedButton(joystick, 4);
  private DebouncedButton button5 = new DebouncedButton(joystick, 5);
  private DebouncedButton button6 = new DebouncedButton(joystick, 6);
  private DebouncedButton button7 = new DebouncedButton(joystick, 7);
  private DebouncedButton button8 = new DebouncedButton(joystick, 8);
  private DebouncedButton button9 = new DebouncedButton(joystick, 9);
  private DebouncedButton button10 = new DebouncedButton(joystick, 10);
  private DebouncedButton button11 = new DebouncedButton(joystick, 11);
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
    controller.back.whileTrue(new IntakeUp());
    controller.back.whileTrue(new IntakeDown());
    controller.bottom.whileTrue(new IntakeIn(0.5));
    controller.bottomLeft.whileTrue(new IntakeOut(0.5));
    controller.bottomRight.whileTrue(new SpinLeft(0.5));
    controller.left.whileTrue(new SpinLeft(0.5));
    controller.right.whileTrue(new SpinRight(0.5));
    controller.leftBumper.whileTrue(new SpinOff());
    
    //Secondary Driver Controls
    button1.whileTrue(new InstantCommand(driveTrain::zeroHeading));
    button2.whileTrue(new AprilTagVission(driveTrain, driveTrain.getLimelight(), Constants.AUTO.XY_PID_CONFIG, Constants.AUTO.THETA_PID_CONFIG));
    button3.whileTrue(new AutoAlign(driveTrain, driveTrain.getLimelight(), driveTrain.getBlinkin(), Constants.AUTO.ALIGN_XY_PID_CONFIG, Constants.AUTO.ALIGN_THETA_PID_CONFIG));
    button4.whileTrue(new IntakeUp());
    button5.whileTrue(new IntakeDown());
    button6.whileTrue(new IntakeIn(0.5));
    button7.whileTrue(new IntakeOut(0.5));
    button8.whileTrue(new SpinLeft(0.5));
    button9.whileTrue(new SpinLeft(0.5));
    button10.whileTrue(new SpinRight(0.5));
    button11.whileTrue(new SpinOff());
  }

  public void createSystemTestButtonBinding(){
    controller.a.toggleOnTrue(new TestSystems(controller.top, controller.bottom, controller.leftY, driveTrain));
  }

  public Command getAutonomousCommand(){
    // return new JanusAuto(driveTrain, autoChooser.getSelected().build());
    return AutoPathPlanner.runAuto(autoPathChooser.getSelected());
  }

}
