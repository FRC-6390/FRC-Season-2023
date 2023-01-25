package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.APRILTAGS;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.sensors.REVBlinkin;
import frc.robot.utilities.sensors.REVColour;
import frc.robot.utilities.sensors.vission.LimeLight;

public class AutoAlign extends CommandBase {

    private LimeLight limeLight;
    private REVBlinkin blinkin;
    private DriveTrain driveTrain;
    private PID xPID, yPID, thetaPID;
    private PIDConfig xyConfig, thetaConfig;

    public AutoAlign(DriveTrain driveTrain, LimeLight limeLight, REVBlinkin blinkin, PIDConfig xyConfig, PIDConfig thetaConfig) {
        this.limeLight = limeLight;
        this.blinkin = blinkin;
        this.driveTrain = driveTrain;
        this.thetaConfig = thetaConfig;
        this.xyConfig = xyConfig;
       

        addRequirements(driveTrain);
    }
  
    @Override
    public void initialize() {
        xPID = new PID(() -> driveTrain.getPose().getX(), null ,xyConfig);
        yPID = new PID(() -> driveTrain.getPose().getY(), null, xyConfig);
        thetaPID = new PID(() -> driveTrain.getPose().getRotation().getRadians(), 0, thetaConfig);
    }
  
    @Override
    public void execute() {
        
        if(!limeLight.hasValidTarget()) return;

        double targetHeightMeters = limeLight.getPipeline() == 0 ? APRILTAGS.getByID((int)limeLight.getAprilTagID()).getHeight() : 0;

        double distance = limeLight.getDistanceFromTarget(targetHeightMeters);

        System.out.println(distance);

        double xOffsetRadians = Math.toRadians(limeLight.getTargetHorizontalOffset());

        double xDistance = Math.cos(xOffsetRadians) * distance;
        double yDistance = Math.sin(xOffsetRadians) * distance;
        double thetaDistance = xOffsetRadians;
        System.out.println(xDistance + " " +yDistance);

        double xSpeed = xPID.calculate(xDistance + driveTrain.getPose().getX());
        double ySpeed = yPID.calculate(yDistance + driveTrain.getPose().getY());
        double thetaSpeed = thetaPID.calculate(thetaDistance + driveTrain.getRotation2d().getRadians());

        boolean xLimit = false;
        boolean yLimit = false;
        boolean thetaLimit = false;
        //lock wheels
        if(xLimit && yLimit && thetaLimit){
            blinkin.setColour(REVColour.Strobe_White);
        }else{
            blinkin.setColour(REVColour.White);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
            System.out.println(chassisSpeeds);
            driveTrain.feedbackDrive(chassisSpeeds);
        }

    }
  
    @Override
    public void end(boolean interrupted) {
        driveTrain.feedbackDrive(new ChassisSpeeds());

    }
  
    @   Override
    public boolean isFinished() {
      return false;
    }


  }