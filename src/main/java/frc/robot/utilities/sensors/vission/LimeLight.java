package frc.robot.utilities.vission;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    private static final String DEFUALT_TABLE = "limelight";
    private NetworkTable limelightTable;
    public NetworkTableEntry tv, tx, ty, ta, ts, tl, tshort, tlong, thor, getpipe, camtran, tc, ledMode, camMode, pipeline, stream, snapshot, crop, tx0, ty0, ta0, ts0, tx1, ty1, ta1, ts1, tx2, ty2, ta2, ts2, cx0, cy0, cx1, cy1;

    enum LedMode{
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);
        private int id;
        private LedMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    enum CameraMode{
        VISION_PROCESSOR(0),
        DRIVER(1);
        private int id;
        private CameraMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    enum StreamMode{
        STANDARD(0),
        PIP_MAIN(1),
        PIP_SECONDARY(2);
        private int id;
        private StreamMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    enum SnapshotMode{
        RESET(0),
        ONCE(1);
        private int id;
        private SnapshotMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public LimeLight(){
        this(DEFUALT_TABLE);
    }

    public LimeLight(String table){
        limelightTable = NetworkTableInstance.getDefault().getTable(table);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        ts = limelightTable.getEntry("ts");
        tl = limelightTable.getEntry("tl");
        tshort = limelightTable.getEntry("tshort");
        tlong = limelightTable.getEntry("tlong");
        thor = limelightTable.getEntry("thor");
        getpipe = limelightTable.getEntry("getpipe");
        camtran = limelightTable.getEntry("camtran");
        tc = limelightTable.getEntry("tc");
        ledMode = limelightTable.getEntry("ledMode");
        camMode = limelightTable.getEntry("camMode");
        pipeline = limelightTable.getEntry("pipeline");
        stream = limelightTable.getEntry("stream");
        snapshot = limelightTable.getEntry("snapshot");
        crop = limelightTable.getEntry("crop");
        tx0 = limelightTable.getEntry("tx0");
        ty0 = limelightTable.getEntry("ty0");
        ta0 = limelightTable.getEntry("ta0");
        ts0 = limelightTable.getEntry("ts0");
        tx1 = limelightTable.getEntry("tx1");
        ty1 = limelightTable.getEntry("ty1");
        ta1 = limelightTable.getEntry("ta1");
        ts1 = limelightTable.getEntry("ts1");
        tx2 = limelightTable.getEntry("tx2");
        ty2 = limelightTable.getEntry("ty2");
        ta2 = limelightTable.getEntry("ta2");
        ts2 = limelightTable.getEntry("ts2");
        cx0 = limelightTable.getEntry("cx0");
        cy0 = limelightTable.getEntry("cy0");
        cx1 = limelightTable.getEntry("cx1");
        cy1 = limelightTable.getEntry("cy1");
    }
    
    
    /**
     * Whether the limelight has any valid targets
     */
    public boolean hasValidTarget(){
        return tv.getDouble(0) == 1;
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getTargetHorizontalOffset(){
        return tx.getDouble(0);
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getTargetVerticalOffset(){
        return ty.getDouble(0);
    }

     /**
     * Target Area (0% of image to 100% of image)
     */
    public double getTargetArea(){
        return ta.getDouble(0);
    }

     /**
     * Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTargetSkew(){
        return ts.getDouble(0);
    }

     /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public double getLatency(){
        return tl.getDouble(0);
    }

     /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     */
    public double getTargetShortestSide(){
        return tshort.getDouble(0);
    }

     /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     */
    public double getTargetLongestSide(){
        return tlong.getDouble(0);
    }

     /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTargetHorizontalSide(){
        return thor.getDouble(0);
    }

     /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTargetVerticalSide(){
        return thor.getDouble(0);
    }

     /**
     * True active pipeline index of the camera (0 .. 9)
     */
    public double getPipeline(){
        return getpipe.getDouble(0);
    }

     /**
     * Results of a 3D position solution, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     */
    public Number[] get3DPosition(){
        return camtran.getNumberArray(null);
    }

    /**
     *  Get the average HSV color underneath the crosshair region as a NumberArray
     */
    public Number[] getAverageHSV(){
        return tc.getNumberArray(null);
    }

    /**
     *  Sets limelight’s LED state
     */
    public void setLedMode(LedMode mode){
        ledMode.setNumber(mode.get());
    }

    /**
     *  Sets limelight’s operation mode
     */
    public void setCameraMode(CameraMode mode){
        camMode.setNumber(mode.get());
    }

    /**
     *  Sets limelight’s streaming mode
     */
    public void setStream(StreamMode mode){
        stream.setNumber(mode.get());
    }

     /**
     *  Sets limelight’s current pipeline (0-9)
     */
    public void setStream(int mode){
        pipeline.setNumber(mode);
    }

    /**
     * Allows users to take snapshots during a match
     */
    public void setSnapshots(int mode){
        snapshot.setNumber(mode);
    }

}
