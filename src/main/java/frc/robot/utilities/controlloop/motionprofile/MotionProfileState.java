package frc.robot.utilities.controlloop.motionprofile;

import java.util.ArrayList;

public record MotionProfileState (double startingPose, double timestamp, ArrayList<MotionProfileComponent> comps){
    
    public MotionProfileState(MotionProfileComponent comp){
        this(0, 0, new ArrayList<MotionProfileComponent>(){{add(comp);}});
    }

    public double getDistanceOnPath(double vi, double a,  double t1){
        return vi*t1 + 0.5*a*(t1*t1);
    }

    public double getPoseAtTime(double t1){
        t1 -= timestamp;
        MotionProfileComponent comp = findRelaventComponent(t1, comps);
        double distance = getDistanceOnPath(comp.vi, comp.a, (t1-comp.timestamp)) + comp.startingPose;
        return startingPose + distance;
    }

    private double calculateSpeed(double vi, double a, double t1){
        return vi + a*t1;
    }

    public double getSpeed(double t1){
        t1 -= timestamp;
        MotionProfileComponent comp = findRelaventComponent(t1, comps);
        double speed = calculateSpeed(comp.vi,comp.a,(t1-comp.timestamp));
        return speed;
    }

    private MotionProfileComponent findRelaventComponent(double time, ArrayList<MotionProfileComponent> comps){
        for (int i = 0; i < comps.size(); i++) {
            MotionProfileComponent comp = comps.get(i);
            if(comp.timestamp >= time) continue;
            return comp;
        }
        return comps.get(comps.size()-1);
    }

    public static MotionProfileState empty(){
        return new MotionProfileState(MotionProfileComponent.empty(0, 0));
    }

    @Override
    public String toString(){
        String timeStampTitle = "TimeStamp: "+timestamp+"\n";
        String poseTitle = "Pose ("+startingPose+")";

        String respone = "";
        int size = comps.size();
        for (int i = 0; i < size; i++) {
            respone += "Speed: "+comps.get(i).toString() + "\n";
        }

        return timeStampTitle + poseTitle + respone;
    }
}