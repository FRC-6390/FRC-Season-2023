// package frc.robot.subsystems;

// import java.util.ArrayList;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.INTAKE;
// import frc.robot.utilities.controlloop.PID;
// import frc.robot.utilities.controlloop.PIDConfig;
// import frc.robot.utilities.controlloop.motionprofile.MotionProfile;
// import frc.robot.utilities.controlloop.motionprofile.MotionProfileComponent;
// import frc.robot.utilities.controlloop.motionprofile.MotionProfileConfig;
// import frc.robot.utilities.controlloop.motionprofile.MotionProfileState;
// import frc.robot.utilities.debug.SystemTest;
// import frc.robot.utilities.debug.SystemTestAction;

// public class Intake extends SubsystemBase implements SystemTest{
    
//     private static TalonFX intakeLeft, intakeRight, arm;
//     private static CANCoder encoder;
//     private static PIDConfig pidConfig;
//     private static PID pid;
//     private static MotionProfileConfig profileConfig;
//     private static MotionProfile profile;
//     private static MotionProfileState currentState;
//     private static double setpoint;
//     private static Timer timer;
    

//     static {
//         intakeLeft = new TalonFX(INTAKE.LEFT_MOTOR);
//         intakeRight = new TalonFX(INTAKE.RIGHT_MOTOR);
//         arm = new TalonFX(INTAKE.ARM);
//         profile = new MotionProfile(profileConfig);
//         pidConfig = new PIDConfig(16, 0, 500).setContinuous(-Math.PI, Math.PI);
//         pid = new PID(pidConfig);
//         setpoint = 0;
//         timer = new Timer();
//         timer.reset();
//         timer.start();
//     }

//     public double getPosition(){
//         return encoder.getPosition()/4096d;
//     }

//     private double getVelocity(){
//         return encoder.getVelocity()/4096d;
//     }

//     public void setSetpoint(double val){
//         setpoint = Math.toRadians(val);
//         calculateMotionProfile();
//     }

//     private double getSetpoint(){
//         return setpoint;
//     }

//     private void calculateMotionProfile(){
//         profile.calculate(currentState, getSetpoint());
//     }

//     private MotionProfileState updateState(){
//         MotionProfileComponent component = new MotionProfileComponent(getPosition(), 0, 0, 0, 0, getVelocity(), getVelocity());
//         return new MotionProfileState(component);
//     }

//     public MotionProfileState getCurrentState(){
//         return currentState;
//     }

//     public void setIntake(double speed){
//         intakeLeft.set(ControlMode.PercentOutput, speed);
//         intakeRight.set(ControlMode.PercentOutput, -speed);
//     }

//     @Override
//     public void periodic() {
//         currentState = updateState();
//         double speed = (profile.getPoseAtTime(timer.get()) + pid.calculate()) / 12;
//         arm.set(ControlMode.PercentOutput, speed);
//     }

//     @Override
//     public ArrayList<SystemTestAction> getDevices() {
//         return new ArrayList<>();
//     }
// }
