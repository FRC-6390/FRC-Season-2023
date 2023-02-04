
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {

  //dummy number for encoder
  public double setpoint = 7000;
  public double speed;
  public PIDController pid;

  public IntakeDown() {
  }

  @Override
  public void initialize() {
    Intake.currentPosition = true;
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void execute() {
    Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));
  }
}
