package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModuleConfig;

public interface Constants {

    public interface DRIVETRAIN{

        String CANBUS = "";

        int PIGEON = 0;

        int REV_PDH = 0;

        Translation2d[] SWERVE_MODULE_LOCATIONS = {ROBOT.FRONT_LEFT, ROBOT.FRONT_RIGHT, ROBOT.BACK_LEFT, ROBOT.BACK_RIGHT};

        int FRONT_LEFT_DRIVE = 0;
        int FRONT_LEFT_ROTATION = 0;
        int FRONT_LEFT_ENCODER = 0;
        int FRONT_RIGHT_DRIVE = 0;
        int FRONT_RIGHT_ROTATION = 0;
        int FRONT_RIGHT_ENCODER = 0;
        int BACK_LEFT_DRIVE = 0;
        int BACK_LEFT_ROTATION = 0;
        int BACK_LEFT_ENCODER = 0;
        int BACK_RIGHT_DRIVE = 0;
        int BACK_RIGHT_ROTATION = 0;
        int BACK_RIGHT_ENCODER = 0;

        int FRONT_LEFT_OFFSET = 0;
        int FRONT_RIGHT_OFFSET = 0;
        int BACK_LEFT_OFFSET = 0;
        int BACK_RIGHT_OFFSET = 0;

        SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_LEFT_DRIVE, false, FRONT_LEFT_ROTATION, false, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(FRONT_RIGHT_DRIVE, false, FRONT_RIGHT_ROTATION, false, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new SwerveModuleConfig(BACK_LEFT_DRIVE, false, BACK_LEFT_ROTATION, false, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET, CANBUS);
        SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new SwerveModuleConfig(BACK_RIGHT_DRIVE, false, BACK_RIGHT_ROTATION, false, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET, CANBUS);

    }

    public interface SWERVEMODULE {
        double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        double MAX_ANGULAR_SPEED_METERS_PER_SECOND = 4;
        double MAX_ACCELERATION_METERS_PER_SECOND = 1;
        double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND = 1;
        double ROTATION_GEAR_RATIO = 1d/(150d/7d);
        double DRIVE_GEAR_RATIO = 1d/(8.14);
        double ROTATION_ENCODER_CONVERSION_RADIANS = ROTATION_GEAR_RATIO * 2 * Math.PI;
        double DRIVE_ENCODER_CONVERSION_METERS = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        PIDConfig ROTATION_PID = new PIDConfig(0.5, 0, 0).setContinuous(-Math.PI, Math.PI);
    }
    
    public interface ROBOT {
        double TRACKWIDTH_METERS = 0;
        double WHEELBASE_METERS = 0;
        Translation2d FRONT_LEFT = new Translation2d(TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d FRONT_RIGHT = new Translation2d(TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
        Translation2d BACK_LEFT = new Translation2d(-TRACKWIDTH_METERS/2, WHEELBASE_METERS/2);
        Translation2d BACK_RIGHT = new Translation2d(-TRACKWIDTH_METERS/2, -WHEELBASE_METERS/2);
    }
}
