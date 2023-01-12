package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.auto.JanusConfig;
import frc.robot.utilities.auto.JanusRoute;
import frc.robot.utilities.auto.JanusRouteFactory;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModuleConfig;

public interface Constants {

    public interface DRIVETRAIN{

        String CANBUS = "rio";

        int PIGEON = 0;

        int REV_PDH = 0;

        Translation2d[] SWERVE_MODULE_LOCATIONS = {ROBOT.FRONT_LEFT, ROBOT.FRONT_RIGHT, ROBOT.BACK_LEFT, ROBOT.BACK_RIGHT};

        int FRONT_LEFT_DRIVE = 0;
        int FRONT_LEFT_ROTATION = 4;
        int FRONT_LEFT_ENCODER = 8;
        int FRONT_RIGHT_DRIVE = 1;
        int FRONT_RIGHT_ROTATION = 5;
        int FRONT_RIGHT_ENCODER = 9;
        int BACK_LEFT_DRIVE = 2;
        int BACK_LEFT_ROTATION = 6;
        int BACK_LEFT_ENCODER = 10;
        int BACK_RIGHT_DRIVE = 3;
        int BACK_RIGHT_ROTATION = 7;
        int BACK_RIGHT_ENCODER = 11;

        double FRONT_LEFT_OFFSET = 1.62447;
        double FRONT_RIGHT_OFFSET = 1.57232;
        double BACK_LEFT_OFFSET = 1.596929;
        double BACK_RIGHT_OFFSET = 1.54779;

        SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_LEFT_DRIVE, false, FRONT_LEFT_ROTATION, false, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_RIGHT_DRIVE, false, FRONT_RIGHT_ROTATION, false, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(BACK_LEFT_DRIVE, false, BACK_LEFT_ROTATION, false, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(BACK_RIGHT_DRIVE, false, BACK_RIGHT_ROTATION, false, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET, CANBUS);

    }

    public interface SWERVEMODULE {
        double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_SPEED_METERS_PER_SECOND_SQUARED = Units.feetToMeters(13.5) * Units.feetToMeters(13.5);
        double MAX_ANGULAR_SPEED_METERS_PER_SECOND = 4;
        double MAX_ACCELERATION_METERS_PER_SECOND = 1;
        double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND = 1;
        double ROTATION_GEAR_RATIO = 1d/(12.8d);// 1d/(150d/7d);
        double DRIVE_GEAR_RATIO = 1d/(8.14);
        double ROTATION_ENCODER_CONVERSION_RADIANS = ROTATION_GEAR_RATIO * 2 * Math.PI;
        double DRIVE_ENCODER_CONVERSION_METERS = (DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS);
        PIDConfig ROTATION_PID = new PIDConfig(0.5, 0, 0).setContinuous(-Math.PI, Math.PI);
    }

    public interface AUTO{
        
        PIDConfig XY_PID_CONFIG = new PIDConfig(0.02, 0, 0);
        PIDConfig THETA_PID_CONFIG = new PIDConfig(0.02, 0, 0).setContinuous(-Math.PI, Math.PI);

        JanusConfig CONFIG = new JanusConfig(SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ACCELERATION_METERS_PER_SECOND, XY_PID_CONFIG, THETA_PID_CONFIG);

        JanusRouteFactory TEST_X_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0).to(-1, 0).to(0, 0);
        JanusRouteFactory TEST_Y_AUTO_PATH = new JanusRouteFactory(CONFIG).to(0, 1).to(0, -1).to(0, 0);
        JanusRouteFactory TEST_XY_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0).to(-1, 0).to(0, 0).to(0, 1).to(0, -1).to(0, 0).to(1,1).to(-1,-1).to(1,-1).to(-1,1).to(0,0);
        JanusRouteFactory TEST_THETA_AUTO_PATH = new JanusRouteFactory(CONFIG).to(0, 0,90).to(0, 0, 270).to(0, 0, 180).to(0, 0, 0);
        JanusRouteFactory TEST_MOVEMENT_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 90).to(-1, 0).to(0, 0, 0).to(0, 1, 180).to(0, -1).to(0, 0, 0).to(1,1, 270).to(-1,-1, 90).to(1,-1, 180).to(-1,1, 90).to(0,0, 0);
        JanusRouteFactory TEXT_COMMAND_1_AUTO_PATH = new JanusRouteFactory(CONFIG).run(null);
        JanusRouteFactory TEXT_COMMAND_2_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 0).run(null).to(0, 0, 0);
        JanusRouteFactory TEXT_COMMAND_3_AUTO_PATH = new JanusRouteFactory(CONFIG).to(1, 0, 0).run(null, true).to(0, 0, 0);

        TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(SWERVEMODULE.MAX_ANGULAR_SPEED_METERS_PER_SECOND, SWERVEMODULE.MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND);
    }
    
    public interface ROBOT {
        double TRACKWIDTH_METERS = 0.61;
        double WHEELBASE_METERS = 0.61;
        Translation2d FRONT_LEFT = new Translation2d(TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d FRONT_RIGHT = new Translation2d(TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        Translation2d BACK_LEFT = new Translation2d(-TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d BACK_RIGHT = new Translation2d(-TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        int BLINKIN_PORT = 1;
    }
    
}
