package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Candle extends CommandBase {
  
  //CANdle LEDS
  CANdle led = new CANdle(Constants.ROBOT.CANDLE_ID, "can");
  String gamePiece;

  public Candle(String gamePiece) {
    this.gamePiece = gamePiece;
  }

  @Override
  public void initialize() {
    if(gamePiece == "Cube" || gamePiece == "cube"){
      led.setLEDs(160, 44, 222);
    } 

    if(gamePiece == "Cone" || gamePiece == "cone"){
      led.setLEDs(247, 200, 59);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
