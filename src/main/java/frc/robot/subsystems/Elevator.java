package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ELEVATOR;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;

public class Elevator extends SubsystemBase {

    private static RelativeEncoder encoder;
    private static CANSparkMax motor;
    private static ShuffleboardTab tab;

    static{
        tab = Shuffleboard.getTab("Elevator");
        motor = new CANSparkMax(12, MotorType.kBrushless);
       // motor = new TalonFX(ELEVATOR.DRIVE_MOTOR);
        encoder =  motor.getEncoder();
        encoder.setPosition(0);
        motor.setIdleMode(IdleMode.kBrake);
    }

    public MotionProfileState getCurrentState(){
        MotionProfileComponent component = new MotionProfileComponent(getPosition(), 0, 0, 0, 0, getVelocity(), getVelocity());
        return new MotionProfileState(component);
    }

    public double getVelocity(){
        return encoder.getVelocity()/2048d * ELEVATOR.GEARBOX_RATIO;
    }

    public double getPosition(){
        return motor.getEncoder().getPosition();
    }

    public void set(double speed){
        motor.set(speed);
    }

    @Override
    public void periodic() {
      //  tab.addDouble("Postion", () -> getPosition());
        
    }
}
