package frc.robot.utilities.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class JanusWaypoint {
    
    private double x, y, theta;
    private Command command;

    public JanusWaypoint(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public JanusWaypoint(Command command){
        this.command = command;
    }

    public JanusWaypoint toNewPostion(double x, double y){
        return new JanusWaypoint(x, y, theta);
    }

    public double getTheta() {
        return theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public Command getCommand() {
        return command;
    }

    public boolean isCommand(){
        return command != null;
    }

}