package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.controlloop.motionprofile.MotionProfile;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileConfig;

public class ElevatorControl extends CommandBase {

    private Elevator elevator;
    private MotionProfileConfig elevatorConfig;
    private MotionProfile motionProfile;
    private double setpoint;
    private Timer timer;

    public ElevatorControl(Elevator elevator, double setpoint, MotionProfileConfig elevatorConfig) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        this.elevatorConfig = elevatorConfig;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        motionProfile = new MotionProfile(elevatorConfig);
        timer = new Timer();
        timer.reset();
        timer.start();
        motionProfile.init(elevator::getPosition);
        motionProfile.calculate(elevator.getCurrentState(), setpoint);
    }

    @Override
    public void execute() {
        double speed = motionProfile.getSpeedsAtTime(timer.get());
        elevator.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
