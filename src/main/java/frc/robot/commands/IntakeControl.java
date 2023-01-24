package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.controlloop.motionprofile.MotionProfile;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileConfig;

public class IntakeControl extends CommandBase {

    private Intake intake;
    private MotionProfileConfig intakeConfig;
    private MotionProfile motionProfile;
    private double setpoint;
    private Timer timer;

    public IntakeControl(Intake intake, double setpoint, MotionProfileConfig intakeConfig) {
        this.intake = intake;
        this.setpoint = setpoint;
        this.intakeConfig = intakeConfig;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        motionProfile = new MotionProfile(intakeConfig);
        timer = new Timer();
        timer.reset();
        timer.start();
        motionProfile.init(intake::getPosition);
        motionProfile.calculate(intake.getCurrentState(), setpoint);
    }

    @Override
    public void execute() {
        double speed = motionProfile.getSpeedsAtTime(timer.get());
        intake.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
