package frc.robot.utilities.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;

public class JanusRouteFactory {
    
    private JanusConfig config;
    private ArrayList<JanusWaypoint> route = new ArrayList<>();


    public JanusRouteFactory(JanusConfig config){
        this.config = config;
    }

    public JanusRouteFactory to(double x, double y){
        return to(x, y, Double.NEGATIVE_INFINITY);
    }

    public JanusRouteFactory to(double x, double y, double theta){
        route.add(new JanusWaypoint(x, y, theta));
        return this;
    }

    public JanusRouteFactory run(Command command){
        route.add(new JanusWaypoint(command));
        return this;
    }

    public JanusRoute build(){
        correctWaypoints(route);

        return new JanusRoute();
    }

    private void correctWaypoints(ArrayList<JanusWaypoint> waypoints){
        for (int i = 0; i < waypoints.size(); i++) {
            JanusWaypoint point = waypoints.get(i);
            //skips Commands
            if(point.isCommand()) continue;
            // adds theta to any points missing it
            if(point.getTheta() == Double.NEGATIVE_INFINITY){
                JanusWaypoint previousPoint = getPreviousWaypoint(i, waypoints);
                if(previousPoint != null) point.setTheta(previousPoint.getTheta());
                else point.setTheta(0);
            }
        }
    }
    
    private JanusWaypoint getPreviousWaypoint(int id, ArrayList<JanusWaypoint> waypoints){
        if(id < 0) return null;
        JanusWaypoint point = waypoints.get(id);
        if(point.isCommand()) return getPreviousWaypoint(id--, waypoints);
        else return point;
    }

}
