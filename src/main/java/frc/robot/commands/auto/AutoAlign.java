package frc.robot.commands.auto;

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

public class AutoAlign extends CommandBase {

    private VissonTracking vissonTracking;
    private DriveTrain driveTrain;
    private PID xPID, yPID, thetaPID;
    private PIDConfig xyConfig, thetaConfig;
    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public AutoAlign(DriveTrain driveTrain, VissonTracking vissonTracking, PIDConfig xyConfig, PIDConfig thetaConfig) {
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

        boolean xLimit = false;
        boolean yLimit = false;
        boolean thetaLimit = false;
        //lock wheels
        if(xLimit && yLimit && thetaLimit){
            vissonTracking.setLEDColour(REVColour.Strobe_White);
        }else{
            vissonTracking.setLEDColour(REVColour.White);
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, driveTrain.getRotation2d());
            driveTrain.feedforwardDrive(chassisSpeeds);
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