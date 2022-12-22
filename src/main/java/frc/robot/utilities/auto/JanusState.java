package frc.robot.utilities.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record JanusState (Pose2d startingPose, double time, JanusVector d, JanusVector a, double t, JanusVector vi, JanusVector vf, double angle){
    

    public double getDistanceOnPath(double vi, double vf,  double t1){
        return (vi + vf)/2 * t1;
    }

    public Pose2d getPoseAtTime(double t1){
        t1 -= time;
        if(t1>t) t1 = t;
        //System.out.println(r);
        double x = getDistanceOnPath(vi.xComp(), vf.xComp(), t1);
        double y = getDistanceOnPath(vi.yComp(), vf.yComp(),t1);
        return new Pose2d(x+startingPose.getX(),y+startingPose.getY(), Rotation2d.fromRadians(angle));
    }

    private double calculateSpeed(double vi, double a, double d){
        return Math.sqrt((vi*vi) + (2 * a * d));
    }

    public ChassisSpeeds getChassisSpeedsAtTime(double t1){
       
        t1 -= time;
        if(t1>t) return new ChassisSpeeds(0,0,0);
        double xSpeed = calculateSpeed(vi.xComp(), a.xComp(), getDistanceOnPath(vi.xComp(), vf.xComp(), t1));
        double ySpeed = calculateSpeed(vi.yComp(), a.yComp(), getDistanceOnPath(vi.yComp(), vf.yComp(), t1));
        double thetaSpeed = 0;

        //System.out.println(this.toString());

        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }

    @Override
    public String toString(){
        return "Time: " + time + " | a: "+a + " | t: "+t + " | d("+d+")" + " | vi("+vi+")" + " | vf("+vf+")";
    }
}
