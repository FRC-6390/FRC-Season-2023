package frc.robot.utilities.sensors;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class REVBlinkin {
    
    private Spark blinkin;
    private REVColour color;
    

    public REVBlinkin(int port){
        blinkin = new Spark(port);
    }

    public void setColour(int id){
        setColour(REVColour.getByID(id));
    }

    public void setColour(REVColour color){
        this.color = color;
        blinkin.set(color.getValue());
    }

    public REVColour getColour(){
        return color;
    }

}
