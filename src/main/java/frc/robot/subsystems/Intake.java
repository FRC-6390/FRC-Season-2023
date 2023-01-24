package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;

public class Intake extends SubsystemBase {

    private static CANCoder positionEncoder;
    private static TalonFX intakeLeft, intakeRight, intakePosition;
    private static ShuffleboardTab tab;

    static{
        tab = Shuffleboard.getTab("Intake");
        intakeLeft = new TalonFX(INTAKE.LEFT_MOTOR);
        intakeRight = new TalonFX(INTAKE.RIGHT_MOTOR);
        intakePosition = new TalonFX(INTAKE.POSITION_MOTOR);
        positionEncoder = new CANCoder(INTAKE.POSITION_ENCODER);
    }

    public MotionProfileState getCurrentState(){
        MotionProfileComponent component = new MotionProfileComponent(getPosition(), 0, 0, 0, 0, getVelocity(), getVelocity());
        return new MotionProfileState(component);
    }

    public double getVelocity(){
        return positionEncoder.getVelocity() * INTAKE.POSITION_GEARBOX_RATIO;
    }

    public double getPosition(){
        return positionEncoder.getPosition()/4096d * INTAKE.POSITION_GEARBOX_RATIO;
    }

    public void set(double speed){
        intakePosition.set(ControlMode.PercentOutput, speed);
    }

    public void rollerSpeed(double speed){
        intakeLeft.set(ControlMode.PercentOutput, speed);
        intakeRight.set(ControlMode.PercentOutput, -speed);
    }

    @Override
    public void periodic() {
        tab.addDouble("Intake Position", () -> getPosition());
    }
}
