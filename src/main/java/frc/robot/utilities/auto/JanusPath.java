package frc.robot.utilities.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class JanusPath {

    private ArrayList<JanusWaypoint> waypoints;
    private boolean isCommand = false;
    private JanusConfig config;
    private ArrayList<ChassisSpeeds> states;

    public JanusPath(ArrayList<JanusWaypoint> waypoints){
        this.waypoints = waypoints;
    }

    public JanusPath(JanusWaypoint waypoint){
        waypoints = new ArrayList<>() {{add(waypoint);}};
        isCommand = true;
    }

    public void calculatePath(){
        states = new ArrayList<>();
        double totalTime = 0;
        double distance;
        double velocityInit = 0;
        double velocityFinal;
        double acceleration = config.maxAccelerationMeters();
        
        for (int i = 0; i < waypoints.size(); i++) {
            JanusWaypoint init = waypoints.get(i);
            JanusWaypoint end = waypoints.size() <= i+1 ? init : waypoints.get(i+1);

            distance = init.distanceFrom(end);

            double time = solveTime(distance, acceleration, velocityInit);
            totalTime += time;

            //velocityFinal = 

            states.add(new ChassisSpeeds(0, 0, 0));
        }
        

        
    }

    public boolean endOfPath(Pose2d pose){
        if(isCommand()) return waypoints.get(0).getCommand().isFinished();
        return false;
    }

    public ChassisSpeeds getSpeedsAtTime(double time){
        return new ChassisSpeeds();
    }

    public void runComman(){
        if(isCommand()) waypoints.get(0).getCommand().schedule();
    }

    public boolean isCommand(){
        return isCommand;
    }

    public Pose2d getPoseAtTime(double time){
        return new Pose2d();
    }

    private double solveTime(double d, double a, double vi){
        double root1, root2;
        double determinant = vi * vi - 4 * a * d;
          root1 = (-vi + Math.sqrt(determinant)) / (2 * a);
          root2 = (-vi - Math.sqrt(determinant)) / (2 * a);

        return root1 > root2 ? root1 : root2;
    }

}
