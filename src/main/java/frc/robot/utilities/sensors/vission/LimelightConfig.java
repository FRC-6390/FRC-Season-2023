package frc.robot.utilities.sensors.vission;

public record LimelightConfig(String table, double mountingAngle, double mountingHeightMeters){
    
    private static String DEFUALT_TABLE = "limelight";

    public LimelightConfig(double mountingAngle, double mountingHeightMeters){
        this(DEFUALT_TABLE, mountingAngle, mountingHeightMeters);
    }

    public static LimelightConfig defualt(){
        return new LimelightConfig(DEFUALT_TABLE, 0, 0);
    }
}
