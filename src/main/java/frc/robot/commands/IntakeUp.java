
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeUp extends CommandBase {

  public double setpoint = 0;
  public double prevError;
  public static PIDController pid;
  public static boolean isDone;

  public IntakeUp() {
  }

  @Override
  public void initialize() {
    isDone = false;
    IntakeDown.isDone = true;
    Intake.setIntakeBoolPosition(true);
    Intake.intakeLift.setNeutralMode(NeutralMode.Brake);
    pid = new PIDController(0.0058, 0.0025, 0);
  }

  @Override
  public void execute() {
    //as long as the limit switch is not triggered run the PID
    if(Intake.getPosition() < 1){
      Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));
      System.out.println("GOING UP");
    } 
    if(Intake.getLimitSwitch() == false){
      System.out.println("RESET");
      Intake.liftEncoder.setPosition(0);
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeState = false;
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
