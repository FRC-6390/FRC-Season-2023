
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;

public class ArmDown extends CommandBase {

  //Dummy number
  public double setpoint = 0;
  public static PIDController pid;
  public static boolean isDone;

  public ArmDown() {
  }

  @Override
  public void initialize() {
    isDone = false;
    ArmUp.isDone = true;
    CommandScheduler.getInstance().cancel(new ArmUp());
    pid = new PIDController(0.014, 0.001, 0);
  }

  @Override
  public void execute() {
      //If within a certain range, end command, else run PID
      if(Arm.getPosition() < 1){
        isDone = true;
        Arm.setLift(0);
      } else{
        System.out.println("GOING DOWN");
        Arm.setLift(pid.calculate(Arm.getPosition(), setpoint));
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
