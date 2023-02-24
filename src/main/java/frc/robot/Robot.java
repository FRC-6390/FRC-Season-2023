package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.OutputRollers;
import frc.robot.commands.SpinWasher;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Intake.liftEncoder.setPosition(0);
    Arm.armEncoder.setPosition(0);
    Elevator.setPosition(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

       //intaking system TRIGGERS
    if(RobotContainer.controller.leftTrigger.getAsDouble() > 0.4){
      new SpinWasher(0.5, 1.0);
      new OutputRollers(0.5, "cube");
    }
    if(RobotContainer.controller.rightTrigger.getAsDouble() > 0.4){
      new OutputRollers(0.5, "cone");
    }
  }

  @Override
  public void disabledInit() {
    Elevator.setPosition(0);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
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
