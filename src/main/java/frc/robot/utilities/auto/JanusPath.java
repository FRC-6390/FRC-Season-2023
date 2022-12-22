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
        JanusVector vi = new JanusVector(0, 0);
        JanusVector vf = new JanusVector(0, 0);
        Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        for (int i = 0; i < waypoints.size()-1; i++) {

            JanusWaypoint init = waypoints.get(i);
            JanusWaypoint end = waypoints.size() <= i+1 ? init : waypoints.get(i+1);
            double angle = init.angleFrom(end);
            JanusVector a = JanusVector.fromResultant(config.maxAccelerationMeters(), angle);
            
            JanusVector d = JanusVector.fromResultant(init.distanceFrom(end), angle);

            double t = solveTime(d.resultant(), a.resultant(), vi.resultant());
            double vfx = calculateFinalVelocity(vi.xComp(), d.xComp(), a.xComp(), t);
            double vfy = calculateFinalVelocity(vi.yComp(), d.yComp(), a.yComp(), t);
            vf = new JanusVector(vfx, vfy);
            if(vf.resultant() > config.maxSpeedMeters()){
                vf = JanusVector.fromResultant(config.maxSpeedMeters(), angle);
                
                double tx = timeAtMaxSpeed(vf.xComp(), a.xComp(), vi.xComp());
                double ty = timeAtMaxSpeed(vf.yComp(), a.yComp(), vi.yComp());
                

                double timeAtMax = tx > ty ? tx : ty;

                
                double dx = distanceAtMaxSpeed(vi.xComp(), vf.xComp(), timeAtMax);
                double dy = distanceAtMaxSpeed(vi.yComp(), vf.yComp(), timeAtMax);

                JanusVector distanceAtMax = new JanusVector(dx, dy);
                
                JanusState state = new JanusState(pose, totalTime, distanceAtMax,  a, timeAtMax, vi, vf, angle);
                totalTime += timeAtMax;
                pose = state.getPoseAtTime(totalTime);
                states.add(state);
                d = JanusVector.fromResultant(d.resultant() - distanceAtMax.resultant(), angle);
                vi = vf;
                a = new JanusVector(0, 0);
                t = d.resultant()/((vf.resultant()+vi.resultant())/2);
            }
            
            JanusState state = new JanusState(pose, totalTime,d, a, t, vi, vf, angle);
            totalTime += t;
            pose = state.getPoseAtTime(totalTime);
            states.add(state);
            
            vi = vf;
        }
    
        Collections.reverse(states);
       for (JanusState js : states) {
           System.out.println(js.toString());
       }
        
    }

    public double calculateFinalVelocity(double vi, double d, double a, double t){
        return Math.sqrt(Math.pow(vi,2) + 2*a*d);
    }

    private double timeAtMaxSpeed(double maxSpeed, double acceleration, double intialSpeed){
        double val = (maxSpeed - intialSpeed)/acceleration;
        return Double.isNaN(val) ? 0 : val;
    }

    private double distanceAtMaxSpeed(double vi, double vf, double t){
        return  (vi + vf)/2 * t;
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
