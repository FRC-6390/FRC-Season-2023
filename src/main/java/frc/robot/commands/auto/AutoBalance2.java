package frc.robot.commands.auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance2 extends CommandBase {

  public PIDController controller;
  public double kp,ki,kd;
  public DriveTrain driveTrain;
  public double pitch;
  public double roll;
  public double speed;
  public double setPoint = 0;
  public double error;
  public double errortot = 0;
  public double errorRate;
  public double prevError;

  public AutoBalance2() {
  }

  @Override
  public void initialize(){
    driveTrain = new DriveTrain();
    kp = 0.01;
    ki = 0.2;
    kd = 0.1;
  }

  @Override
  public void execute()
  {
    pitch = driveTrain.getPitch();
    if(error < 10)
    {
      errortot += error * ki;
    }

    errorRate = error - prevError;
    error = setPoint - pitch;
    speed = (error * kp) + errortot + (errorRate * ki);
    double xSpeed = 0.0;
    double ySpeed = speed;
    double thetaSpeed = 0.0;
    

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, driveTrain.getRotation2d());

    
    driveTrain.drive(chassisSpeeds);
    prevError = error;
    if(error == 0)
    {
      driveTrain.lockWheels();
    }
    else
    {
      driveTrain.unlockWheels();
    }
    System.out.println(pitch + ' ' + error + ' ' + errortot + ' ' + errorRate + ' ' + prevError + ' ' + speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
