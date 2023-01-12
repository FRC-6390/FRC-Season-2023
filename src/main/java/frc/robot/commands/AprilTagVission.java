package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VissonTracking;
import frc.robot.utilities.auto.JanusRoute;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.sensors.REVColour;

public class AprilTagVission extends CommandBase {

    private VissonTracking vissonTracking;
    private DriveTrain driveTrain;
    private PID xPID, yPID, thetaPID;
    private PIDConfig xyConfig, thetaConfig;
    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public AprilTagVission(DriveTrain driveTrain, VissonTracking vissonTracking, PIDConfig xyConfig, PIDConfig thetaConfig) {
        this.vissonTracking = vissonTracking;
        this.driveTrain = driveTrain;
        this.thetaConfig = thetaConfig;
        this.xyConfig = xyConfig;
        xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND);

        addRequirements(driveTrain, vissonTracking);
    }
  
    @Override
    public void initialize() {
        xPID = new PID(() -> driveTrain.getPose().getX(), vissonTracking::getOffset ,xyConfig);
        yPID = new PID(() -> driveTrain.getPose().getY(), 0, xyConfig);
        thetaPID = new PID(() -> driveTrain.getPose().getRotation().getRadians(), 0, thetaConfig);
    }
  
    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(xPID.calculate()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
        double ySpeed = yLimiter.calculate(yPID.calculate()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
        double thetaSpeed = thetaLimiter.calculate(thetaPID.calculate()) * SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

        boolean xLimit = true;
        boolean yLimit = false;
        boolean thetaLimit = false;
        System.out.println(vissonTracking.getOffset());
        //lock wheels
        if(vissonTracking.getOffset() > 1){
          System.out.println("True");
          ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(1, 0, 0, driveTrain.getRotation2d());
          driveTrain.drive(chassisSpeeds);
        } else {
          System.out.println("False");
          driveTrain.stopWheels();
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