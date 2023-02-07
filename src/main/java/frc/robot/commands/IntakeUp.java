
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeUp extends CommandBase {

  //Dummy number
  public double setpoint = 0;
  public double speed;
  public double prevError;
  public static PIDController pid;

  //boolean to decide when the command should end
  public boolean isDone;

  public IntakeUp() {
  }

  @Override
  public void initialize() {
    isDone = false;
    Intake.currentPosition = false;

    //Dummy Numbers
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void execute() {
    //as long as the limit switch is not triggered run the PID
    if(Intake.getLimitSwitch() != false){
      Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));

      //DUMMY NUMBER 0
      if(Intake.getPosition() > 0){
        isDone = true;
      }
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
