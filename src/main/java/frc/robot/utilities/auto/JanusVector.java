package frc.robot.utilities.auto;

public record JanusVector(double xComp, double yComp, double resultant) {

    public JanusVector (double xComp, double yComp){
        this(xComp, yComp, getResultant(xComp, yComp));  
    }

    public static JanusVector fromResultant(double resultant, double angle){
        return new JanusVector(Math.cos(angle)*resultant, Math.sin(angle)*resultant, resultant);
    }

    public static double getResultant(double xComp, double yComp){
        return Math.sqrt((xComp*xComp) + (yComp*yComp));
    }

    public static double roundTo(double n, double places){
        return Math.round(n*Math.pow(10, places))/Math.pow(10, places);
    }
    
    @Override
    public String toString(){
        return "x: " + xComp + " | y: " + yComp + " | r: " + resultant;
    }
}
