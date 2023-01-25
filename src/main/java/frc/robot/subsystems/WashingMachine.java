package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WASHINGMACHINE;
import frc.robot.utilities.debug.SystemTest;
import frc.robot.utilities.debug.SystemTestAction;

public class WashingMachine extends SubsystemBase implements SystemTest {

    private static TalonFX motor;

    static{
        motor = new TalonFX(WASHINGMACHINE.MOTOR_ID);
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public ArrayList<SystemTestAction> getDevices() {
        return new ArrayList<>();
    }
}
