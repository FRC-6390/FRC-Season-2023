package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;

public class Elevator extends SubsystemBase {

    private static CANCoder encoder;
    private static TalonFX motor;
    private static ShuffleboardTab tab;
    public static DigitalInput elevatorLimitSwitch;

    static{
        tab = Shuffleboard.getTab("Elevator");
        motor = new TalonFX(ELEVATOR.DRIVE_MOTOR, "can");
        encoder = new CANCoder(ELEVATOR.ENCODER, "can");
        elevatorLimitSwitch = new DigitalInput(Constants.ELEVATOR.LIMIT_SWITCH); 
        encoder.setPosition(0);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public MotionProfileState getCurrentState(){
        MotionProfileComponent component = new MotionProfileComponent(getPosition(), 0, 0, 0, 0, getVelocity(), getVelocity());
        return new MotionProfileState(component);
    }

    public double getVelocity(){
        return encoder.getVelocity()/2048d * ELEVATOR.GEARBOX_RATIO;
    }

    public static double getPosition(){
        return encoder.getPosition();
    }

    public static void setPosition(double position){
        encoder.setPosition(position);
    }

    public static void set(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

      //Get value of the intake lift limit switch
    public static boolean getLimitSwitch(){
        //false for triggered, otherwise true
        return elevatorLimitSwitch.get();
    }

    @Override
    public void periodic() {
        tab.addDouble("Postion", () -> getPosition());
    }
}
