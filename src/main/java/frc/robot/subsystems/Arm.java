package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

/** A robot arm subsystem that moves with a motion profile. */
public class Arm extends ProfiledPIDSubsystem {
  
  public static  TalonFX armMotor = new TalonFX(Constants.ARM.ARM_MOTOR);
  private final CANCoder armEncoder = new CANCoder(Constants.ARM.ARM_ENCODER);
  public static TalonFX leftRoller, rightRoller;
  public static boolean currentPosition;

  //use link too find calculated values https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A15%2C%22u%22%3A%22cm%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A360%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A270%2C%22u%22%3A%22deg%22%7D
  private final ArmFeedforward armFeedforward = new ArmFeedforward(0.26, 1.8, 0, 0);

  public Arm() {
    super(
        new ProfiledPIDController(0.01,0,0, new TrapezoidProfile.Constraints(Math.PI, Math.PI * Math.PI)),
        0
    );

    // Start arm at rest in neutral position
    setGoal(0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);

    // Add the feed forward to the PID output to get the motor outputR
    armMotor.set(ControlMode.PercentOutput, output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return armEncoder.getPosition()/4096d;
  }


  //Sets rollers
  public static void setRollers(double speed) {
    leftRoller.set(ControlMode.PercentOutput, speed);
    rightRoller.set(ControlMode.PercentOutput, -speed);
  }

  public static double getRollersVoltage() {
    double volts = leftRoller.getMotorOutputVoltage() + rightRoller.getMotorOutputVoltage() / 2;
    return volts;
  }

  //Sets Arm Lift
  public static void setLift(double speed) {
    leftRoller.set(ControlMode.PercentOutput, speed);
  }
}