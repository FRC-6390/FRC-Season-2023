package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

public final class AutoPathPlanner {
    
    public AutoPathPlanner() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    static PIDConstants XY_PID = new PIDConstants(0, 0, 0);
    static PIDConstants THETA_PID = new PIDConstants(0, 0, 0);

    //events to trigger commands while in autonomous
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("Intake", Commands.print("Intake Command Triggered")),
        Map.entry("Intake In", Commands.print("Intake In Command Triggered")),
        Map.entry("Intake Out", Commands.print("Intake Out Command Triggered")),
        Map.entry("Linear Extension L1 Out", Commands.print("Linear Extension L1 Out Command Triggered")),
        Map.entry("Linear Extension L2 Out", Commands.print("Linear Extension L2 Out Command Triggered")),
        Map.entry("Linear Extension L3 Out", Commands.print("Linear Extension L3 Out Command Triggered")),
        Map.entry("Linear Extension In", Commands.print("Linear Extension In Command Triggered"))
    ));

    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            RobotContainer.driveTrain::getPose,
            RobotContainer.driveTrain::resetOdometry,
            XY_PID,
            THETA_PID,
            RobotContainer.driveTrain::drive,
            eventMap,
            RobotContainer.driveTrain
    );

    public static CommandBase runAuto(String autoSelector) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(autoSelector, new PathConstraints(4, 3)));
    }
}