package frc.robot.utilities.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.controlloop.PIDConfig;

public class Main {

    static PIDConfig xyconfig = new PIDConfig(0, 0, 0);
    static PIDConfig thetaconfig = new PIDConfig(0, 0, 0);
    static JanusConfig config = new JanusConfig(1, 1, xyconfig, xyconfig);
    static JanusRouteFactory factory = new JanusRouteFactory(config).to(1, 0).to(1,1);
    
    public static void main(String[] args) {
        JanusRoute route = factory.build();

        route.init(Pose2d::new);
        
        //System.out.println("Time: "+0.1+" - "+ route.calculate(0.05)); 
        for (double i = 0d; i <= 2.6d; i+=0.1d) {
            System.out.printf("Time: %.2f - %s - %s%n",i,route.calculate(i), route.getPose(i)); 
        }
    }
}
