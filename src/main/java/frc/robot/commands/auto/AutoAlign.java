package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VissionTracking;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.sensors.REVColour;

public class AutoAlign extends CommandBase {

    private VissionTracking vissionTracking;
    private DriveTrain driveTrain;
    private PID xPID, yPID, thetaPID;
    private PIDConfig xyConfig, thetaConfig;
    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public AutoAlign(DriveTrain driveTrain, VissionTracking vissonTracking, PIDConfig xyConfig, PIDConfig thetaConfig) {
        this.vissionTracking = vissonTracking;
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
        xPID = new PID(() -> driveTrain.getPose().getX(), null ,xyConfig);
        yPID = new PID(() -> driveTrain.getPose().getY(), null, xyConfig);
        thetaPID = new PID(() -> driveTrain.getPose().getRotation().getRadians(), 0, thetaConfig);
    }
  
    @Override
    public void execute() {

        double distance = vissionTracking.getDistance();

        double xDistance = Math.cos(Math.toRadians(vissionTracking.getXOffset())) * distance;
        double yDistance = Math.sin(Math.toRadians(vissionTracking.getXOffset())) * distance;

        double xSpeed = xLimiter.calculate(xPID.calculate(xDistance + driveTrain.getPose().getX())) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
        double ySpeed = yLimiter.calculate(yPID.calculate(yDistance + driveTrain.getPose().getY())) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
        double thetaSpeed = thetaLimiter.calculate(thetaPID.calculate()) * SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND;

        boolean xLimit = false;
        boolean yLimit = false;
        boolean thetaLimit = false;
        //lock wheels
        if(xLimit && yLimit && thetaLimit){
            vissionTracking.setLEDColour(REVColour.Strobe_White);
        }else{
            vissionTracking.setLEDColour(REVColour.White);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
            driveTrain.feedbackDrive(chassisSpeeds);
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