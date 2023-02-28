package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriverControl;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.GoingDown;
import frc.robot.commands.GoingHigh;
import frc.robot.commands.GoingLow;
import frc.robot.commands.GoingMid;
import frc.robot.commands.OutputRollers;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeRollers;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.SpinWasher;
import frc.robot.commands.TapeVission;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoPathPlanner;
import frc.robot.subsystems.DriveTrain;
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

  public RobotContainer() {

    driveTrain.shuffleboard();
    driveTrain.setDefaultCommand(new DriverControl(driveTrain, controller.leftX, controller.leftY, controller.rightX));

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
    controller.back.whileTrue(new AutoBalance(driveTrain));

    // controller.a.whileTrue(new AprilTagVission(driveTrain, driveTrain.getLimelight(), Constants.AUTO.XY_PID_CONFIG, Constants.AUTO.THETA_PID_CONFIG));
    // controller.b.whileTrue(new AutoAlign(driveTrain, driveTrain.getLimelight(), driveTrain.getBlinkin(), Constants.AUTO.ALIGN_XY_PID_CONFIG, Constants.AUTO.ALIGN_THETA_PID_CONFIG));
    controller.x.whileTrue(new TapeVission(driveTrain, driveTrain.getLimelight(), Constants.AUTO.XY_PID_CONFIG, Constants.AUTO.THETA_PID_CONFIG));

    controller.a.onTrue(new GoingDown());
    controller.b.onTrue(new GoingMid());
    controller.y.onTrue(new GoingHigh());

    


    // controller.leftBumper.onTrue();
    controller.rightBumper.onTrue(new IntakeUp());

    controller.leftBumper.whileTrue(new ParallelCommandGroup(new IntakeDown(), new SpinWasher(0.5, 0.9)));

    controller.leftStick.whileTrue(new OutputRollers(0.5, "cone", 0));
    controller.rightStick.whileTrue(new OutputRollers(0.5, "cube", 0));

    //secondary driver controls on Logitech Controller
    joystick.seven.whileTrue(new ParallelCommandGroup(new OutputRollers(0.5, "cube", 0), new SpinWasher(0.5, 0)));
    joystick.eight.whileTrue(new ParallelCommandGroup(new OutputRollers(0.5, "cone", 0),  new SpinWasher(0.5, 0)));

    joystick.nine.onTrue(new GoingLow());
    joystick.ten.onTrue(new GoingDown());

    //outfeeding intake
    joystick.eleven.whileTrue(new IntakeRollers(-0.5));

  }

  public Command getAutonomousCommand(){
    return AutoPathPlanner.runAuto(autoPathChooser.getSelected());
  }



  // public void createSystemTestButtonBinding(){
  //   controller.a.toggleOnTrue(new TestSystems(controller.top, controller.bottom, controller.leftY, driveTrain));
  // }
}
