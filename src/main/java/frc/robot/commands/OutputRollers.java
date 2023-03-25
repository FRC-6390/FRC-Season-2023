package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class OutputRollers extends CommandBase {
 
  public double speed;
  public boolean isDone;
  public String gamePiece;
  public double rotations;

  public OutputRollers(double speed, String gamePiece, double rotations) {
    this.speed = speed;
    this.gamePiece = gamePiece;
    this.rotations = rotations;
  }

  
  @Override
  public void initialize() {
    isDone = false;
    Arm.resetOutputRollerPosition();
  }

  
  @Override
  public void execute() {
    System.out.println(Math.abs(Arm.getOutputRollerPosition()));
    if(rotations != 0){
      if(Math.abs(Arm.getOutputRollerPosition()) > rotations){
        isDone = true;
      } 
    }
      
    // System.out.println(Arm.getRollersVoltage());
    if(gamePiece == "Cube" || gamePiece == "cube"){
      CANdle led = new CANdle(Constants.ROBOT.CANDLE_ID, "can");
      led.setLEDs(125, 35, 250);
      Arm.setRoller(speed);
    } 

    if(gamePiece == "Cone" || gamePiece == "cone"){
      CANdle led = new CANdle(Constants.ROBOT.CANDLE_ID, "can");
      led.setLEDs(255, 188, 5);
      Arm.setRoller(-speed);
    } 
  }

  
  @Override
  
  public void end(boolean interrupted) {
    //Stops the motors
    Arm.setRoller(0);
  }

  
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
