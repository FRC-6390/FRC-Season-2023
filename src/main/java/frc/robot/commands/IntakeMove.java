package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;

public class IntakeMove extends CommandBase {

   public double setpoint = 0;
   public double speed;
   public PIDConfig config;
   public PID pid;
   public boolean isDone;
 
   public IntakeMove(double setpoint) {
    this.setpoint = setpoint;
    config = new PIDConfig(16, 0.2, 1000).setContinuous(-Math.PI, Math.PI).setILimit(0);
   }
 
   @Override
   public void initialize() {
     isDone = false;    
     Intake.liftEncoder.setPosition(0);
     Intake.intakeLift.setNeutralMode(NeutralMode.Brake);
     Intake.currentPosition = true;
     pid = new PID(() -> Intake.liftEncoder.getPosition(), () -> setpoint, config);
   }
 
   @Override
   public void execute() {
    speed = pid.calculate();
    Intake.setLift(convertRange(speed, new double[]{-12,12}, new double[]{-1,1}));
   }
 
   @Override
   public void end(boolean interrupted) {
     Intake.setLift(0);  
   }
 
   @Override
   public boolean isFinished() {
     return isDone;
   }

   public double convertRange(double input, double[] oldRange,  double[] newRange){
    return (((input - (oldRange[0])) * (newRange[1] - (newRange[0]))) / (oldRange[1] - (oldRange[0]))) + newRange[0];
  }
 }
 