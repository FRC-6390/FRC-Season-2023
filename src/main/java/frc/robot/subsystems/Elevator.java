package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;

public class Elevator extends SubsystemBase {

    private static CANCoder encoder;
    private static TalonFX motor;

    static{
        motor = new TalonFX(ELEVATOR.DRIVE_MOTOR);
        encoder = new CANCoder(ELEVATOR.ENCODER);
    }

    public MotionProfileState getCurrentState(){
        MotionProfileComponent component = new MotionProfileComponent(getPosition(), 0, 0, 0, 0, getVelocity(), getVelocity());
        return new MotionProfileState(component);
    }

    public double getVelocity(){
        return encoder.getVelocity() * ELEVATOR.GEARBOX_RATIO;
    }

    public double getPosition(){
        return encoder.getPosition()/4096d * ELEVATOR.GEARBOX_RATIO;
    }

    public void set(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {

    }
}
