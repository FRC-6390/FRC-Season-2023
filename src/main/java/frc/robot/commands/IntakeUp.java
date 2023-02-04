
package frc.robot.commands;

import javax.xml.namespace.QName;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeMahdi;

public class IntakeUp extends CommandBase {
  /** Creates a new IntakeArmCommand. */
  public double error;
  public double errorSum;
  public double errorRate;
  public double kp, ki, kd;
  
  //Dummy number
  public double setpoint = 0;
  public double speed;
  public double prevError;
  public static PIDController pid;

  public IntakeUp() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    IntakeMahdi.pos = true;
    //Dummy Numbers
    
    pid = new PIDController(0.01, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // error =  IntakeMahdi.getPosition() - setpoint;
    // if(Math.abs(error) < 100)
    // {
    //   errorSum += error;
    // }
    // errorRate = prevError - error;
    // speed = (error * kp) + (errorSum * ki) + (errorRate * kd);
    // prevError = error;

    IntakeMahdi.setLift(pid.calculate(IntakeMahdi.getPosition(), setpoint));
  }
}
