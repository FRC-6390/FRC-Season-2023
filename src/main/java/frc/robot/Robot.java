package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  TalonFX motor = new TalonFX(4, "canTest");

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    motor.set(ControlMode.PercentOutput, 0.5);
    
  }

  @Override
  public void disabledInit() {}

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
    m_robotContainer.createSystemTestButtonBinding();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
