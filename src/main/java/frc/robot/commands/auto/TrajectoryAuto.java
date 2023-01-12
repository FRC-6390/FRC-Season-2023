package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.subsystems.DriveTrain;

public class TrajectoryAuto extends SequentialCommandGroup{

    public TrajectoryAuto(final DriveTrain driveTrain) {

        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND,
            Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND_SQUARED
        ).setKinematics(driveTrain.kinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(2, 0),                    
                new Translation2d(2, 2)),
                new Pose2d(10, 0, Rotation2d.fromDegrees(0)
            ),trajectoryConfig
        );

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(0.001, 0, 0);
        PIDController yController = new PIDController(0.001, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            1, 0, 0, Constants.AUTO.THETA_CONTROLLER_CONSTRAINTS
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            driveTrain::getPose,
            driveTrain.kinematics,
            xController,
            yController,
            thetaController,
            driveTrain::setModuleStates,
            driveTrain
        );

        // 5. Add some init and wrap-up, and return everything
        addCommands(
            new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> driveTrain.stopWheels())
        );

    }
}
