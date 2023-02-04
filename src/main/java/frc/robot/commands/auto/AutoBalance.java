package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {

  DriveTrain driveTrain;
  ChassisSpeeds speeds;
  PIDController xController = new PIDController(0.05, 0, 0);
  PIDController yController = new PIDController(0.05, 0, 0);
  double xSpeed, ySpeed;

  public AutoBalance(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double gyroPitch = driveTrain.getPitch();//front and back
    double gyroRoll = Math.abs(driveTrain.getRoll());//left and right

    //true if they need to start auto balancing
    boolean pitchStatus = Math.abs(driveTrain.getPitch()) > 0.6;
    boolean rollStatus = Math.abs(driveTrain.getRoll()) > 0.6;

    xSpeed = xController.calculate(gyroPitch);
    ySpeed = xController.calculate(gyroRoll);
  
    if(pitchStatus && rollStatus){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, driveTrain.getRotation2d());
    } else if(pitchStatus){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0.0, 0, driveTrain.getRotation2d());
    } else if(rollStatus){
      System.out.println("ROLLING");
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, ySpeed, 0, driveTrain.getRotation2d());
    }
    driveTrain.drive(speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
