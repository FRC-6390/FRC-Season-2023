package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

public final class AutoPathPlanner {

    static AutoBalance autoBalance = new AutoBalance(RobotContainer.driveTrain);

    public AutoPathPlanner() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    static PIDConstants XY_PID = new PIDConstants(1, 0, 0);
    static PIDConstants THETA_PID = new PIDConstants(4.9, 0, 0);

    //events to trigger commands while in autonomous
    private static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
        Map.entry("Spin Washer", Commands.print("Spinning Washing Machine Command Triggered")),
        Map.entry("Intake Up", Commands.print("Intake Up Command Triggered")),
        Map.entry("Intake Down", Commands.print("Intake Down Command Triggered")),
        Map.entry("Intake In", Commands.print("Intake In Command Triggered")),
        Map.entry("Intake Out", Commands.print("Intake Out Command Triggered")),
        Map.entry("Linear Extension L1 Out", Commands.print("Linear Extension L1 Out Command Triggered")),
        Map.entry("Linear Extension L2 Out", Commands.print("Linear Extension L2 Out Command Triggered")),
        Map.entry("Linear Extension L3 Out", Commands.print("Linear Extension L3 Out Command Triggered")),
        Map.entry("Linear Extension In", Commands.print("Linear Extension In Command Triggered")),
        Map.entry("Auto Balance", new AutoBalance(RobotContainer.driveTrain)) 
    ));

    private static final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            RobotContainer.driveTrain::getPose,
            RobotContainer.driveTrain::resetOdometry,
            XY_PID,
            THETA_PID,
            RobotContainer.driveTrain::drive,
            eventMap,
            true,
            RobotContainer.driveTrain
    );

    public static CommandBase runAuto(String autoSelector) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(autoSelector, new PathConstraints(3, 1.5)));
    }
}
