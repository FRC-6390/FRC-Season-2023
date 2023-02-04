package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WashingMachine extends SubsystemBase {

  public static TalonFX washingMotor;
  public WashingMachine() {}

  static{
    washingMotor = new TalonFX(Constants.WASHING_MACHINE.MOTOR_ID);
  }

  @Override
  public void periodic() {

  }

  public static void set(double speed){
    washingMotor.set(ControlMode.PercentOutput, speed);
  }


}

