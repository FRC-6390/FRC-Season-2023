package frc.robot.utilities.auto;

import java.util.ArrayList;

import javax.management.openmbean.ArrayType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record JanusState (Pose2d startingPose, double timestamp, ArrayList<JanusComponent> xComps, ArrayList<JanusComponent> yComps){
    
    public JanusState(JanusComponent xComp, JanusComponent yComp){
        this(null, 0, new ArrayList<JanusComponent>(){{add(xComp);}}, new ArrayList<JanusComponent>(){{add(yComp);}});
    }


    public double getDistanceOnPath(double vi, double a,  double t1){
        return vi*t1 + 0.5*a*(t1*t1);
    }

    public Pose2d getPoseAtTime(double t1){
        t1 -= timestamp;
        JanusComponent xComp = findRelaventComponent(t1, xComps);
        JanusComponent yComp = findRelaventComponent(t1, yComps);

        double x = getDistanceOnPath(xComp.vi, xComp.a, (t1-xComp.timestamp)) + xComp.startingPose;
        double y = getDistanceOnPath(yComp.vi, yComp.a, (t1-yComp.timestamp)) + yComp.startingPose;
        return new Pose2d(x+startingPose.getX(),y+startingPose.getY(), Rotation2d.fromRadians(0));
    }

    private double calculateSpeed(double vi, double a, double t1){
        return vi + a*t1;
    }

    public ChassisSpeeds getChassisSpeedsAtTime(double t1){
        t1 -= timestamp;
        JanusComponent xComp = findRelaventComponent(t1, xComps);
        JanusComponent yComp = findRelaventComponent(t1, yComps);
        

        double xSpeed = calculateSpeed(xComp.vi,xComp.a,(t1-xComp.timestamp));
        double ySpeed = calculateSpeed(yComp.vi,yComp.a, (t1-yComp.timestamp));
        double thetaSpeed = 0;

        //System.out.println(this.toString());

        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }

    private JanusComponent findRelaventComponent(double time, ArrayList<JanusComponent> comps){
        for (int i = 0; i < comps.size(); i++) {
            JanusComponent comp = comps.get(i);
            if(comp.timestamp >= time) continue;
            return comp;
        }
        return comps.get(comps.size()-1);
    }

    public static JanusState empty(){
        return new JanusState(new JanusComponent(0, 0, 0, 0, 0, 0, 0), new JanusComponent(0, 0, 0, 0, 0, 0, 0));
    }

    @Override
    public String toString(){
        String timeStampTitle = "TimeStamp: "+timestamp+"\n";
        String poseTitle = startingPose == null ? "Pose(null,null), null\n" : "Pose ("+startingPose.getX()+","+startingPose.getY()+"), " + startingPose.getRotation().getDegrees()+"\n";

        String respone = "";
        int size = xComps.size() > yComps.size() ? xComps.size() : yComps.size();
        int xDataPrevious = 0;
        int yDataPrevious = 0;
        for (int i = 0; i < size; i++) {
            String xData = xComps.size() > i ? "X: "+xComps.get(i).toString(): String.format("%"+xDataPrevious+"s", " ");
            String yData = yComps.size() > i ? " Y: "+yComps.get(i).toString(): String.format("%"+yDataPrevious+"s", " ");
            xDataPrevious = xData.length();
            yDataPrevious = yData.length();
            respone += xData + yData + "\n"; 
        }

        return timeStampTitle + poseTitle + respone;
    }
}
