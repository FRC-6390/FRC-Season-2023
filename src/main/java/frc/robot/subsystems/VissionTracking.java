package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ROBOT;
import frc.robot.utilities.sensors.REVBlinkin;
import frc.robot.utilities.sensors.REVColour;
import frc.robot.utilities.sensors.vission.LimeLight;
import frc.robot.utilities.sensors.vission.LimeLight.LedMode;

public class VissionTracking extends SubsystemBase{

    private static REVBlinkin blinkin;
    private static LimeLight limelight;

    static{
        blinkin = new REVBlinkin(ROBOT.BLINKIN_PORT);
        limelight = new LimeLight();

    }

    public double getDistance(){
        return limelight.getDistanceFromTarget(0);
    }

    public void turnOFFLEDS(){
        limelight.setLedMode(LedMode.OFF);
    }

    public double getXOffset(){
        return limelight.getTargetHorizontalOffset();
    }

    public double getYOffset(){
        return limelight.getTargetVerticalOffset();
    }

    public void setLEDColour(int colour){
        blinkin.setColour(colour);
    }

    public void setLEDColour(REVColour colour){
        blinkin.setColour(colour);
    }

    public REVColour getLEDColour(){
        return blinkin.getColour();
    }

    @Override
    public void periodic() {
       
    }
    
}
