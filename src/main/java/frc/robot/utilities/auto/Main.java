package frc.robot.utilities.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.controlloop.motionprofile.MotionProfile;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileConfig;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;

public class Main {

    static PIDConfig xyconfig = new PIDConfig(0.1, 0, 0).setContinuous(-180, 180);
    static PIDConfig thetaconfig = new PIDConfig(0, 0, 0);
    static JanusConfig config = new JanusConfig(3, 2, 3, 2, xyconfig, thetaconfig);
    static JanusRouteFactory factory = new JanusRouteFactory(config).to(2,1).to(0,0).to(-3,3,180);
    static double pos = 160;
    public static void main(String[] args) {
        //JanusRoute route = factory.build();

        //route.init(Pose2d::new);

        MotionProfileConfig config = new MotionProfileConfig(1, 0.5, thetaconfig);

        MotionProfile motionProfile = new MotionProfile(config);

        MotionProfileState currenState = new MotionProfileState(new MotionProfileComponent(0, 0, 0, 0, 0, 0, 0));

        motionProfile.calculate(currenState, 1);

        for (double i = 0d; i <= 10; i+=0.1d) {
            System.out.printf("Time: %.2f - %s - %s%n",i,motionProfile.getSpeedsAtTime(i), motionProfile.getPoseAtTime(i)); 
        }
               
        // for (double i = 0d; i <= 10; i+=0.1d) {
        //     System.out.printf("Time: %.2f - %s - %s%n",i,route.calculate(i), route.getPose(i)); 
        // }

       //PIDController
        
        // PID pid = new PID(xyconfig);

        // pid.setSetpoint(140);
        // pid.setMeasurement(() -> pos);

        // for (double i = 0d; i <= 10; i+=0.1d) {            
        //     double move = pid.calculate();
        //     pos += move;
        //     if(pos > 180) pos -= 360;
        //     if(pos < -180) pos += 360;

        //     System.out.println(pos );
        // }
    }
}
