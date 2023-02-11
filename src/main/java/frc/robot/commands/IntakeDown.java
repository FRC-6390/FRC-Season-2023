
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {

  //dummy number for encoder
  public double setpoint = -110;
  public double speed;
  public PIDController pid;
  public boolean isDone;

  public IntakeDown() {
  }

  @Override
  public void initialize() {
    isDone = false;
    
    Intake.liftEncoder.setPosition(0);
    //setting to coast when it is going down to allow it some room to move if it was hit
    Intake.intakeLift.setNeutralMode(NeutralMode.Brake);
    Intake.currentPosition = true;
    pid = new PIDController(0.0015, 0.0, 0);
  }

  @Override
  public void execute() {
    
    //If within a certain range, end command, else run PID
    if(Intake.getPosition() < -109 && Intake.getPosition() >= -111){
      isDone = true;
    } else{
      System.out.println("GOING DOWN");
      Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));
      // Intake.setLift(0.3);
    }
  }

  @Override
  public void end(boolean interrupted) {
    //setting to break when it all the way up to avoid shaking
    Intake.setLift(0);
    
    System.out.println("DONE");
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
