
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {

  //dummy number for encoder
  public double setpoint = 7000;
  public double speed;
  public PIDController pid;
  public boolean isDone;

  public IntakeDown() {
  }

  @Override
  public void initialize() {
    isDone = false;

    //setting to coast when it is going down to allow it some room to move if it was hit
    Intake.intakeLift.setNeutralMode(NeutralMode.Coast);
    
    Intake.currentPosition = true;
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void execute() {
    Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));

    //DUMMY NUMBER 0
    if(Intake.getPosition() > 0){
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    //setting to break when it all the way up to avoid shaking
    Intake.intakeLift.setNeutralMode(NeutralMode.Brake);
    Intake.setLift(0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
