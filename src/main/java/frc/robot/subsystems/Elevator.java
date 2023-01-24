package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;
import frc.robot.utilities.debug.SystemTest;
import frc.robot.utilities.debug.SystemTestAction;

public class Elevator extends SubsystemBase implements SystemTest {

    private static CANCoder encoder;
    private static TalonFX motor;
    private static ShuffleboardTab tab;

    static{
        tab = Shuffleboard.getTab("Elevator");
        motor = new TalonFX(ELEVATOR.DRIVE_MOTOR);
        encoder = new CANCoder(ELEVATOR.ENCODER);
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

    public double getPosition(){
        return encoder.getPosition();
    }

    public void set(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
      //  tab.addDouble("Postion", () -> getPosition());
        
    }

    @Override
    public ArrayList<SystemTestAction> getDevices() {
        return new ArrayList<>();
    }
}
