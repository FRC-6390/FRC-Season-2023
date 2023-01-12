package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.debug.SystemTest;
import frc.robot.utilities.swerve.SwerveModule;

public class DriveTrain extends SubsystemBase implements SystemTest{

  private static SwerveModule[] swerveModules;
  //private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;
  public static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;
  private static ShuffleboardTab tab, autoTab;
  private static Field2d gameField;

  static {
    tab = Shuffleboard.getTab("Drive Train");
    autoTab = Shuffleboard.getTab("Auto");
    gameField = new Field2d();
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG, tab);
    swerveModules[1] = new SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG, tab);
    swerveModules[2] = new SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG, tab);
    swerveModules[3] = new SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG, tab);

    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);
   
    //pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] SwervePositions = {swerveModules[0].getPostion(), swerveModules[1].getPostion(), swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);

    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()), SwervePositions);
    pose = new Pose2d();

  }

  public void init(){
   // pdh.clearStickyFaults();
    zeroHeading();
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public double getRoll(){
    return gyro.getRoll(); 
  }

  public double getPitch(){
    return gyro.getPitch(); 
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
  }

  public Pose2d getPose(){
    return pose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
    }
  }

  private SwerveModulePosition[] getModulePostions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public void feedbackDrive(ChassisSpeeds speeds){
    feedbackSpeeds = speeds;
  }

  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
    }
  }

  public void lockWheels(){
    double angle = Math.toRadians(getHeading()-45d);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
      swerveModules[i].setToAngle(angle);
    }
  }

  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

  @Override
  public void periodic() {
    

    double xSpeed = chassisSpeeds.vxMetersPerSecond + feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond + feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond + feedbackSpeeds.omegaRadiansPerSecond;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    
    setModuleStates(states);

    odometry.update(getRotation2d(), getModulePostions());
    pose = odometry.getPoseMeters();
    gameField.setRobotPose(pose);
    autoTab.add(gameField);
    autoTab.add("Odometry", pose);
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public boolean testFunctionality(int id, double speed) {

    if(id < 0 || id >= swerveModules.length*2) {
      for (int i = 0; i < swerveModules.length; i++) {
        swerveModules[i].stop();
      }
      return true;
    }
    
    if(id >= 4) {
      swerveModules[id-4].setRotationMotor(speed);
      //stop previous module
      swerveModules[id-5].setRotationMotor(0);
    }
    else {
      swerveModules[id].setDriveMotor(speed);
      //stop previous module
      if(id > 0) swerveModules[id-1].setDriveMotor(0);
    }
  
    return false;
  }

  @Override
  public int getNumberOfComponents() {
    return swerveModules.length*2;
  }
}


