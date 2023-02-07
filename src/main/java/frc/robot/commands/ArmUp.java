
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ArmUp extends CommandBase {

  //Dummy number
  public double setpoint = 0;
  public double prevError;
  public static PIDController pid;

  //boolean to decide when the command should end
  public boolean isDone;

  public ArmUp() {
  }

  @Override
  public void initialize() {
    isDone = false;
    Arm.currentPosition = false;
    Arm.armMotor.setNeutralMode(NeutralMode.Brake);
    pid = new PIDController(0.01, 0, 0);
  }

  @Override
  public void execute() {
    //Runs the PID
    Arm.setLift(pid.calculate(Arm.getPosition(), setpoint));

    //If the switch is triggered or if the encoder value is 0, end command 
    if(Intake.getPosition() == 0){
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    Arm.setLift(0);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
