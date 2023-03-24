
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {

  //dummy number for encoder
  public double setpoint = Constants.INTAKE.SETPOINT_DOWN;
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
    pid = new PIDController(0.0035, 0.0, 0);
  }

  @Override
  public void execute() {
    System.out.println(Intake.getPosition());
    //If within a certain range, end command, else run PID
    if(Intake.getPosition() < setpoint + 20){
      isDone = true;
    } else{
      System.out.println("Going Down");
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
