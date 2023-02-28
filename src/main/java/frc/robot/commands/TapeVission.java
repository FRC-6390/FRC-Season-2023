package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.sensors.vission.LimeLight;
import frc.robot.utilities.sensors.vission.LimeLight.LedMode;

public class TapeVission extends CommandBase {

    private LimeLight limelight;
    private DriveTrain driveTrain;
    private PIDConfig xyConfig, thetaConfig;
    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private PIDController yController;
    private ChassisSpeeds chassisSpeeds;
    private PIDController pid;

    public TapeVission(DriveTrain driveTrain, LimeLight limelight, PIDConfig xyConfig, PIDConfig thetaConfig) {
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        this.thetaConfig = thetaConfig;
        this.xyConfig = xyConfig;
        xLimiter = new SlewRateLimiter(Units.feetToMeters(8));
        yLimiter = new SlewRateLimiter(Units.feetToMeters(8));
        thetaLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
      pid = new PIDController(0.05, 0.007, 0);
      LimeLight.setLedMode(LedMode.ON);
    }
  
    @Override
    public void execute() {
        double xController = 10 * Math.toRadians(limelight.getTargetHorizontalOffset());
        double yController = 10 * Math.toRadians(limelight.getTargetVerticalOffset());

        // double xSpeed = yLimiter.calculate(xController);
        // double ySpeed = yLimiter.calculate(yController);
        
        
        double xSpeed = pid.calculate(limelight.getTargetHorizontalOffset(), 0);
        double ySpeed = pid.calculate(limelight.getTargetVerticalOffset(), 0);

        if((limelight.getTargetVerticalOffset() < 0.5) && (limelight.getTargetVerticalOffset() > -0.5)){
          chassisSpeeds = new ChassisSpeeds(0, 0, 0);
          driveTrain.drive(chassisSpeeds);
        }
        
        // else if(limelight.getTargetHorizontalOffset() < -0.5){
        //   chassisSpeeds = new ChassisSpeeds(xSpeed, 0.0, 0);
        //   driveTrain.drive(chassisSpeeds);
        // }
        else {
          chassisSpeeds = new ChassisSpeeds(0.0, -ySpeed, 0);
          driveTrain.drive(chassisSpeeds);
        }

    }
  
    @Override
    public void end(boolean interrupted) {
      LimeLight.setLedMode(LedMode.OFF);
    }
  

    @Override
    public boolean isFinished() {
      return false;
    }
}