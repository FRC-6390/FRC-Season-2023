package frc.robot.utilities.debug;

public interface SystemTest {

    boolean testFunctionality(int id, double speed);
    
    int getNumberOfComponents();
}
