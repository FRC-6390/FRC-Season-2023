package frc.robot.utilities.controlloop.motionprofile;

public class MotionProfileComponent {
        
    double startingPose;
    double timestamp;
    double d;
    double a;
    double t;
    double vi;
    double vf;

    public MotionProfileComponent(double startingPose, double timestamp, double d, double a, double t, double vi, double vf){
        this.startingPose = startingPose;
        this.d = d;
        this.a = a;
        this.t = t;
        this.vi = vi;
        this.vf = vf;
        this.timestamp = timestamp;
    }
    
    public static MotionProfileComponent empty(double startingPose, double timestamp){
        return new MotionProfileComponent(startingPose,timestamp, 0, 0, 0, 0, 0);
    }
    @Override
    public String toString(){
        return String.format("pose - %.2f | ts - %.2f | d - %.2f | a - %.2f | t - %.2f | vi - %.2f | vf - %.2f", startingPose,timestamp,d,a,t,vi,vf);
    }
}