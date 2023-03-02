package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.sensors.vission.LimeLight;
import frc.robot.utilities.sensors.vission.LimeLight.LedMode;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Intake.liftEncoder.setPosition(0);
    Arm.armEncoder.setPosition(0);
    Elevator.setPosition(0);
    Arm.resetOutputRollerPosition();
    LimeLight.setLedMode(LedMode.OFF);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Elevator.setPosition(0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
    Arm.resetOutputRollerPosition();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    // m_robotContainer.createSystemTestButtonBinding();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
