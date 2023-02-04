// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WashingMachineMahdi extends SubsystemBase {
  /** Creates a new WashingMachineMahdi. */
  public static TalonFX spinMotor;
  public WashingMachineMahdi() {}

  static
  {
    spinMotor = new TalonFX(Constants.WASHINGMACHINE.MOTOR_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public static void set(double speed)
  {
    spinMotor.set(ControlMode.PercentOutput, speed);
  }


}

