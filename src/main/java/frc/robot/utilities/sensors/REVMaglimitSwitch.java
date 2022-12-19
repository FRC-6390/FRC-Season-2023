package frc.robot.utilities.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class REVMaglimitSwitch extends DigitalInput {

    private boolean reversed;

    public REVMaglimitSwitch(int channel) {
        this(channel,false);
    }

    public REVMaglimitSwitch(int channel, boolean reversed) {
        super(channel);
        this.reversed = reversed;
    }

    @Override
    public boolean get() {
        return reversed ? !super.get() : super.get();
    }
    
}