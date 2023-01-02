package frc.robot.utilities.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.controlloop.PIDConfig;

public class Main {

    static PIDConfig xyconfig = new PIDConfig(0, 0, 0);
    static PIDConfig thetaconfig = new PIDConfig(0, 0, 0);
    static JanusConfig config = new JanusConfig(3, 1, xyconfig, xyconfig);
    static JanusRouteFactory factory = new JanusRouteFactory(config).to(2,0);
    
    public static void main(String[] args) {
        JanusRoute route = factory.build();

        route.init(Pose2d::new);
               
        for (double i = 0d; i <= 20; i+=0.1d) {
           System.out.printf("Time: %.2f - %s - %s%n",i,route.calculate(i), route.getPose(i)); 
        }
    }
}
