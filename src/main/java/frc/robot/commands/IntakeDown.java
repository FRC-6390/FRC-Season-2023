
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {

  //dummy number for encoder
  public double setpoint = -13000;
  public PIDController pid;
  public static boolean isDone;

  public IntakeDown() {
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().cancel(new IntakeUp());
    isDone = false;
    IntakeUp.isDone = true;
    Intake.intakeLift.setNeutralMode(NeutralMode.Brake);
    pid = new PIDController(0.000025, 0.0, 0);
  }

  @Override
  public void execute() {
    System.out.println(Intake.getPosition());
    //If within a certain range, end command, else run PID
    if(Intake.getPosition() < -14200){
      isDone = true;
    } else{
      Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));
    }
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setLift(0);
  }

  @Override
  public boolean isFinished() {
    // return isDone;
    return false;
  }
}
