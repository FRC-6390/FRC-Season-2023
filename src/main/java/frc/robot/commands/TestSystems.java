package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.controller.DebouncedButton;
import frc.robot.utilities.debug.SystemTest;

public class TestSystems extends CommandBase{

    private SystemTest[] subsystems;
    private DebouncedButton forwards, backwards;
    private DoubleSupplier speed;

    public TestSystems(DebouncedButton forwards, DebouncedButton backwards, DoubleSupplier speed, SystemTest... subsystems){
        this.subsystems = subsystems;
        this.forwards = forwards;
        this.backwards = backwards;
        this.speed = speed;
        System.out.println("test constructed");

    }

    @Override
    public void initialize() {
        System.out.println("test init");

    }
    
    @Override
    public void execute() {
        System.out.println("test exe");

        for (int i = 0; i < subsystems.length;) {
            for (int j = 0; j < subsystems.length;) {
                //test next device in sub
                if(forwards.debounced()) j++;
                //test previous device in sub
                if(backwards.debounced()) j--;
                //go to previous sub
                if(j < 0) i-=2;
                
                //if done continue to next sub
                if(subsystems[i].testFunctionality(j, speed.getAsDouble()/2)) i++;
            }
        }
    }


    @Override
    public void end(boolean interrupted) {
      
    }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
