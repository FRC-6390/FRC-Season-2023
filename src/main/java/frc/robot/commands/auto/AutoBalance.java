package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.auto.JanusRoute;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;

public class AutoBalance extends CommandBase {

    private DriveTrain driveTrain;
    private PID pitchPID, rollPID;
    private PIDConfig pitchConfig, rollConfig;
    private SlewRateLimiter xLimiter, yLimiter;

    public AutoBalance(DriveTrain driveTrain, PIDConfig pitchConfig, PIDConfig rollConfig) {
        this.driveTrain = driveTrain;
        this.pitchConfig = pitchConfig;
        this.rollConfig = rollConfig;
        xLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        yLimiter = new SlewRateLimiter(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        addRequirements(driveTrain);
    }
  
    @Override
    public void initialize() {
        pitchPID = new PID(driveTrain::getPitch, 0, pitchConfig);
        rollPID = new PID(driveTrain::getRoll, 0, rollConfig);
    }
  
    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(pitchPID.calculate()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
        double ySpeed = xLimiter.calculate(rollPID.calculate()) * SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND;
        
        boolean xLimit = false;
        boolean yLimit = false;

        //lock wheels
        if(xLimit && yLimit){
            driveTrain.lockWheels();
        }else{
            driveTrain.unlockWheels();
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, driveTrain.getRotation2d());
            driveTrain.feedforwardDrive(chassisSpeeds);
        }

    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }


  }