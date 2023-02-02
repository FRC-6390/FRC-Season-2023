// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  /** Creates a new AutoBalance2. */
  public AutoBalance2() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    kp = 0.01;
    ki = 0.2;
    kd = 0.1;
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
