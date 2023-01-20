package frc.robot.commands.auto;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;

public class AutoBalance extends CommandBase {

    private DriveTrain driveTrain;
    private PID pitchPID, rollPID;
    private PIDConfig pitchConfig, rollConfig;

    public AutoBalance(DriveTrain driveTrain, PIDConfig pitchConfig, PIDConfig rollConfig) {
        this.driveTrain = driveTrain;
        this.pitchConfig = pitchConfig;
        this.rollConfig = rollConfig;
        addRequirements(driveTrain);
    }
  
    @Override
    public void initialize() {
        pitchPID = new PID(driveTrain::getPitch, 0, pitchConfig);
        rollPID = new PID(driveTrain::getRoll, 0, rollConfig);
    }
  
    @Override
    public void execute() {
        double xSpeed = pitchPID.calculate(); 
        double ySpeed = rollPID.calculate();
        
        boolean xLimit = Math.abs(xSpeed) < 0.05;
        boolean yLimit = Math.abs(ySpeed) < 0.05;

        System.out.println("Pitch("+xLimit+"): "+driveTrain.getPitch()+" - "+xSpeed+" | Roll:("+yLimit+") "+driveTrain.getRoll() + " - "+ySpeed);

        //lock wheels
        if(xLimit && yLimit){
            driveTrain.lockWheels();
        }else{
            driveTrain.unlockWheels();
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 0, driveTrain.getRotation2d());
            //driveTrain.feedbackDrive(chassisSpeeds);
        }

    }
  
    @Override
    public void end(boolean interrupted) {
        driveTrain.unlockWheels();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }


  }