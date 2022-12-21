package frc.robot.utilities.auto;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class JanusPath {

    private ArrayList<JanusWaypoint> waypoints;
    private boolean isCommand = false;
    private JanusConfig config;
    private ArrayList<JanusState> states;

    public JanusPath(ArrayList<JanusWaypoint> waypoints, JanusConfig config){
        this.waypoints = waypoints;
        this.config = config;
    }

    public JanusPath(JanusWaypoint waypoint, JanusConfig config){
        this(new ArrayList<>() {{add(waypoint);}}, config);
        isCommand = true;
    }

    public void calculatePath(){
        states = new ArrayList<>();
        double totalTime = 0;
        double distance = 0;
        double velocityInit = 0;
        double velocityFinal = 0;
        double acceleration = 0;
        double previousAngle = 0;
        Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        for (int i = 0; i < waypoints.size()-1; i++) {
            
            acceleration = config.maxAccelerationMeters();
            JanusWaypoint init = waypoints.get(i);
            JanusWaypoint end = waypoints.size() <= i+1 ? init : waypoints.get(i+1);
            double angle = init.angleFrom(end);
            distance = init.distanceFrom(end);

            double yComp = velocityInit*Math.sin(previousAngle);
            double xComp = velocityInit*Math.cos(previousAngle);
            velocityInit = Math.sqrt((xComp*xComp)+(yComp*yComp));
            double time = solveTime(distance, acceleration, velocityInit);
            velocityFinal = Math.sqrt(Math.pow(velocityInit,2) + 2*acceleration*distance);
           // System.out.println(velocityFinal);
           

            if(velocityFinal > config.maxSpeedMeters()){
                velocityFinal = config.maxSpeedMeters();
                double timeAtMax = timeAtMaxSpeed(velocityFinal, acceleration, velocityInit);
                
                double distanceAtMax = distanceAtMaxSpeed(velocityFinal, acceleration, velocityInit);
               // System.out.println(distanceAtMax);

                JanusState state = new JanusState(pose, totalTime, acceleration, distanceAtMax, timeAtMax, velocityInit, velocityFinal, angle);
                totalTime += timeAtMax;
                pose = state.getPoseAtTime(totalTime);
                states.add(state);
                distance -= distanceAtMax;
                velocityInit = velocityFinal;
                acceleration = 0;
                time = distance/((velocityFinal+velocityInit)/2);
            }

            
            JanusState state = new JanusState(pose, totalTime, acceleration, distance, time, velocityInit, velocityFinal, angle);
            totalTime += time;
            pose = state.getPoseAtTime(totalTime);
            states.add(state);
           // System.out.println(states.get(states.size()-1).toString());
            System.out.println(pose);
            velocityInit = velocityFinal;
            previousAngle = angle;
        }
    
        Collections.reverse(states);
       for (JanusState js : states) {
         System.out.println(js.toString());
       }
        
    }

    private double timeAtMaxSpeed(double maxSpeed, double acceleration, double intialSpeed){
        return (maxSpeed - intialSpeed)/acceleration;
    }

    private double distanceAtMaxSpeed(double maxSpeed, double acceleration, double intialSpeed){
        return (Math.pow(maxSpeed,2) - Math.pow(intialSpeed,2))/(2*acceleration);
    }

    public boolean endOfPath(Pose2d pose){
        if(isCommand()) return waypoints.get(0).getCommand().isFinished();
        return false;
    }

    private JanusState findRelaventState(double time){
        for (int i = 0; i < states.size(); i++) {
            JanusState state = states.get(i);
            if(state.time() > time) continue;
            return state;
        }
        return states.get(states.size()-1);
    }

    public ChassisSpeeds getSpeedsAtTime(double time){
        return findRelaventState(time).getChassisSpeedsAtTime(time);
    }

    public void runComman(){
        if(isCommand()) waypoints.get(0).getCommand().schedule();
    }

    public boolean isCommand(){
        return isCommand;
    }

    public Pose2d getPoseAtTime(double time){
        return findRelaventState(time).getPoseAtTime(time);
    }


    // A = a, B = vi, C = d
    private double solveTime(double d, double a, double vi){
        double root1, root2;
        double determinant = vi * vi - 4d * a * -d;
        if (determinant == 0.0){
            root1 = -vi/(2.0*a);
            return root1;
        }
        root1 = (-vi + Math.pow(determinant,0.5)) / (2.0 * a);
        root2 = (-vi - Math.pow(determinant,0.5)) / (2.0 * a);

        return root1 > root2 ? root1 : root2;
    }

}
