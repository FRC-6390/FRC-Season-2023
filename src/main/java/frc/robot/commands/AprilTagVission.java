package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VissionTracking;
import frc.robot.utilities.auto.JanusRoute;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.sensors.REVColour;
import frc.robot.utilities.sensors.vission.LimeLight;

public class AprilTagVission extends CommandBase {

    private LimeLight limelight;
    private DriveTrain driveTrain;
    private PID xPID, yPID, thetaPID;
    private PIDConfig xyConfig, thetaConfig;
    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private PIDController yController;
    private ChassisSpeeds chassisSpeeds;

    public AprilTagVission(DriveTrain driveTrain, LimeLight limelight, PIDConfig xyConfig, PIDConfig thetaConfig) {
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        this.thetaConfig = thetaConfig;
        this.xyConfig = xyConfig;
        xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        double xController = 3 * Math.toRadians(limelight.getTargetVerticalOffset());
        double yController = 3 * Math.toRadians(limelight.getTargetHorizontalOffset());

        double xSpeed = yLimiter.calculate(xController);
        double ySpeed = yLimiter.calculate(yController);
        
        //lock wheels
        if((limelight.getTargetHorizontalOffset() > 1) || (limelight.getTargetVerticalOffset() > 1)){
          System.out.println(yController + ": " + ySpeed);
          chassisSpeeds = new ChassisSpeeds(xSpeed, -ySpeed, 0);
          driveTrain.drive(chassisSpeeds);
        }else {
          chassisSpeeds = new ChassisSpeeds(0, 0, 0);
          driveTrain.drive(chassisSpeeds);
        }

    }
  
    @Override
    public void end(boolean interrupted) {
        
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }


  }