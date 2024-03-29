
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmUp extends CommandBase {

  public double setpoint;
  public static PIDController pid;
  public static boolean isDone;
  public boolean overidePID;

  public ArmUp(double setpoint, boolean overidePID) {
    this.setpoint = setpoint;
    this.overidePID = overidePID;
  }

  @Override
  public void initialize() {
    isDone = false;
    ArmDown.isDone = true;
    if(overidePID == true){
      pid = new PIDController(0.1, 0.05, 0);
      System.out.println("____________---------------____________----------------_____________");
      System.out.println("------------_______________------------________________-------------");
    } else {
      pid = new PIDController(0.015, 0.005, 0);
    }
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
