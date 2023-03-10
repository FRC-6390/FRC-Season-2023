
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmUp extends CommandBase {

  //Dummy number 190 high 120 normal
  public double setpoint;
  public static PIDController pid;
  public static boolean isDone;

  public ArmUp(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    isDone = false;
    ArmDown.isDone = true;
    pid = new PIDController(0.024, 0.0075, 0);
  }

  @Override
  public void execute() {
    if(Arm.getPosition() < setpoint){
      Arm.setLift(pid.calculate(Arm.getPosition(), setpoint));
      System.out.println("GOING UP");
    } else {
      Arm.setLift(0);
      isDone = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    Arm.setLift(0.059);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
