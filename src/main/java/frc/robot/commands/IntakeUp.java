
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeUp extends CommandBase {

  public double setpoint = -2;
  public static PIDController pid;
  public static boolean isDone;

  public IntakeUp() {
  }

  @Override
  public void initialize() {
    isDone = false;
    IntakeDown.isDone = true;
    Intake.intakeLift.setNeutralMode(NeutralMode.Brake);
    pid = new PIDController(0.006, 0.0, 0);
  }

  @Override
  public void execute() {
    //as long as the limit switch is not triggered run the PID
    if(Intake.getPosition() < -2){
      Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));
      System.out.println("GOING UP");
    } 

    if(Intake.getLimitSwitch() == false){
      System.out.println("RESET");
      Intake.liftEncoder.setPosition(0);
      // isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
