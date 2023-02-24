package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class OutputRollers extends CommandBase {
 
  public double speed;
  public boolean isDone = false;
  public String gamePiece;

  public OutputRollers(double speed, String gamePiece) {
    this.speed = speed;
    this.gamePiece = gamePiece;
  }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
    // if(Arm.getRollersVoltage() > 4){
    //   Arm.setRoller(0);
    //   isDone = true;
    // } 
    // else {
      
      System.out.println(Arm.getRollersVoltage());
      if(gamePiece == "Cube" || gamePiece == "cube"){
        CANdle led = new CANdle(Constants.ROBOT.CANDLE_ID, "can");
        led.setLEDs(125, 35, 250);
        Arm.setRoller(speed);
      } 
  
      if(gamePiece == "Cone" || gamePiece == "cone"){
        CANdle led = new CANdle(Constants.ROBOT.CANDLE_ID, "can");
        led.setLEDs(125, 35, 250);
        Arm.setRoller(-speed);
      }
    // }
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
