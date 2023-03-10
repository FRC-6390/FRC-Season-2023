package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;
import frc.robot.commands.GoingDown;
import frc.robot.commands.GoingHigh;
import frc.robot.commands.GoingLow;
import frc.robot.commands.GoingMid;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeRollers;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.OutputRollers;
import frc.robot.commands.SpinWasher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
        Map.entry("Reset", new InstantCommand(RobotContainer.driveTrain::zeroHeading)),
        Map.entry("Wait", new WaitCommand(1)),
        Map.entry("Spin Washer", new SpinWasher(0.5, 0.9)),
        Map.entry("Intake Up", new IntakeUp()),
        Map.entry("Intake Down", new IntakeDown()),
        Map.entry("Intake In", new IntakeRollers(0.9)),
        Map.entry("Intake Out", new IntakeRollers(-0.9)),
        Map.entry("Intake Stop", new IntakeRollers(0.0)),
        Map.entry("Cube", new OutputRollers(0.3, "cube", 15000)),
        Map.entry("Cone", new OutputRollers(0.3, "cone", 15000)),
        // Map.entry("L1", new GoingLow()),
        // Map.entry("L2", new GoingMid()),
        // Map.entry("L3", new GoingHigh()),
        // Map.entry("L0", new GoingDown()),
        
        Map.entry("L2", new ArmUp(50)),
        Map.entry("L0", new ArmDown()),
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
        //1.5   0.8
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(autoSelector, new PathConstraints(1.3, 0.8)));
    }
}
