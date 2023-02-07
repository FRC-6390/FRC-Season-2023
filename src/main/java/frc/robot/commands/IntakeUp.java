
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
    //Sets lift to brake mode
    Intake.intakeLift.setNeutralMode(NeutralMode.Brake);

    //Dummy Numbers
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void execute() {
    //as long as the limit switch is not triggered run the PID
    if(Intake.getLimitSwitch() == true)
    {
      //Runs the PID
      Intake.setLift(pid.calculate(Intake.getPosition(), setpoint));

      //If the switch is triggered or if the encoder value is 0, end command 
      if(Intake.getPosition() == 0 || Intake.getLimitSwitch() == false){
        isDone = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    
    Intake.setLift(0);
    Intake.liftEncoder.setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
