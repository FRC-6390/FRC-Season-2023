package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;

public class DriverControl extends CommandBase {

  private DriveTrain driveTrain;
  private DoubleSupplier xInput, yInput, thetaInput;
  private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  public DriverControl(DriveTrain driveTrain, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput) {
    this.driveTrain = driveTrain;
    this.xInput = xInput;
    this.yInput = yInput;
    this.thetaInput = thetaInput;
    xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
  
    double xSpeed = xLimiter.calculate(xInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double ySpeed = yLimiter.calculate(yInput.getAsDouble()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
    double thetaSpeed = thetaLimiter.calculate(thetaInput.getAsDouble()) * SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;


    // xSpeed = 0.5;
    // ySpeed = 0;
    // thetaSpeed = 0;

  

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, driveTrain.getRotation2d());

    driveTrain.drive(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
