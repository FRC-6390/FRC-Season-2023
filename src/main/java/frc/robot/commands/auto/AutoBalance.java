package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {

  DriveTrain driveTrain;
  ChassisSpeeds speeds;
  PIDController xController = new PIDController(0.06, 0.001, 0);
  PIDController yController = new PIDController(0.06, 0.001, 0);
  double pitchSpeed, rollSpeed;
  boolean isDone;

  public AutoBalance(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
  }

  @Override
  public void initialize() {
    isDone = false;
  }

  @Override
  public void execute() {
    double gyroPitch = driveTrain.getPitch();//front and back
    double gyroRoll = driveTrain.getRoll();//left and right

    //true if they need to start auto balancing
    boolean pitchStatus = Math.abs(driveTrain.getPitch()) > 0.5;
    boolean rollStatus = Math.abs(driveTrain.getRoll()) > 0.5; //must calibrate to 0.6 ish

    pitchSpeed = xController.calculate(gyroPitch);
    rollSpeed = xController.calculate(gyroRoll);

    if(pitchStatus && rollStatus){
      if(pitchSpeed > rollSpeed){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(pitchSpeed, 0.0, 0.0, driveTrain.getRotation2d());
      } else{
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(rollSpeed, 0.0, 0.0, driveTrain.getRotation2d());
      }
      
    } else if(pitchStatus){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(pitchSpeed, 0.0, 0.0, driveTrain.getRotation2d());
    } else if(rollStatus){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(rollSpeed, 0.0, 0.0, driveTrain.getRotation2d());
    }

    //false is when it is balanced
    if(Math.abs(driveTrain.getPitch()) < 0.6 && Math.abs(driveTrain.getRoll()) < 0.6){
      isDone = true;
    } else{
      driveTrain.drive(speeds);
    }
    System.out.println(isDone);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
