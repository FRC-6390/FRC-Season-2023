
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeUp extends CommandBase {

  //Dummy number
  public double setpoint = 0;
  public double speed;
  public double prevError;
  public static PIDController pid;

  public IntakeUp() {
  }

  @Override
  public void initialize() {
    Intake.currentPosition = false;

    //Dummy Numbers
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void execute() {
    Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));
  }
}
