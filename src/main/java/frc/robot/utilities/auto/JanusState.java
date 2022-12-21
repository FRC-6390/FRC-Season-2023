package frc.robot.utilities.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record JanusState (Pose2d startingPose, double time, double a, double d, double t, double vi, double vf, double angle){
    

    public double getDistanceOnPath(double t1){
        return vi*t1 + (0.5)*a*(t1*t1);
    }

    public Pose2d getPoseAtTime(double t1){
        t1 -= time;
        if(t1>t) t1 = t;
        double r = getDistanceOnPath(t1);
        //System.out.println(r);
        double x = Math.cos(angle)*(r);
        double y = Math.sin(angle)*(r);
        return new Pose2d(x+startingPose.getX(),y+startingPose.getY(), Rotation2d.fromRadians(angle));
    }

    public ChassisSpeeds getChassisSpeedsAtTime(double t1){
       
        t1 -= time;
        if(t1>t) return new ChassisSpeeds(0,0,0);
        double rSpeed = vi + a*t1;
        double xSpeed = Math.cos(angle)*rSpeed;
        double ySpeed = Math.sin(angle)*rSpeed;
        double thetaSpeed = 0;

        //System.out.println(this.toString());

        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }
}
