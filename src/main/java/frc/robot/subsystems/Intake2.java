// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake2 extends SubsystemBase {
  /** Creates a new Intake2. */
  public static PWMSparkMax intakeMotor;
  public Intake2() 
  {

  }

  static
  {
    intakeMotor = new PWMSparkMax(0);
  }

  public void set(double speed)
  {
    intakeMotor.set(speed);
  }
  
}
