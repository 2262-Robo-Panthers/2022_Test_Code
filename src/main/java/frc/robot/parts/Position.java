package frc.robot.parts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.other.PartRunner;

public class Position extends Part{
    //--------------Settings--------------//
    //pose
    public Pose2d startPose = new Pose2d(0,0,new Rotation2d());
    //distance
    public int kEncoderCPR = 2048;
    public double kGearRatio = 20.8333;
    public double kWheelDiameterMeters = Units.inchesToMeters(4);
    //imu
    public double imuDrift = 0.4;//set in degrees/second

    //--------------Variables--------------//
    private ADIS16470_IMU imu;
    private long lastImuReset;
    private DifferentialDriveOdometry odometry;
    private Drive drive;
    private double kEncoderDistancePerPulse;

    public Position(Drive drive, PartRunner runner){
        super(runner);
        setDrive(drive);
    }

    public void setDrive(Drive drive){
        this.drive = drive;
    }

    ///////////////
    //init/update//
    ///////////////
    public void init(){
        //make imu
        imu = new ADIS16470_IMU();
        //reset
        updateEncoderDistancePerPulse();
        resetAngle();
        drive.resetEncoders();
        //make odometry
        odometry = new DifferentialDriveOdometry(getLatestIMUAngle(), startPose);
    }

    public void updateEncoderDistancePerPulse(){
        kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR / (double) kGearRatio);
    }

    ///////
    //imu//
    ///////
    public double getLatestIMuAngleDegrees(){
        return imu.getAngle() - (((double)(System.currentTimeMillis() - lastImuReset) / 1000.0) * imuDrift);
    }

    public Rotation2d getLatestIMUAngle(){
        return Rotation2d.fromDegrees(getLatestIMuAngleDegrees());
    }
    
    public void resetAngle(){
        imu.reset();
        lastImuReset = System.currentTimeMillis();
    }

    ////////////
    //odometry//
    ////////////
    public void setRobotPose(Pose2d poseMeters){
        odometry.resetPosition(poseMeters, getLatestIMUAngle());
        drive.resetEncoders();
    }
    
    public void run(){
        odometry.update(getLatestIMUAngle(), getLeftDistance(), getRightDistance());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public Rotation2d getAngle(){
        return odometry.getPoseMeters().getRotation();
    }

    public void stop(){}

    ////////////
    //Distance//
    ////////////
    public double getLeftDistance(){
        return drive.getLeftTicks() * kEncoderDistancePerPulse;
      }
    
    public double getRightDistance(){
        return drive.getRightTicks() * kEncoderDistancePerPulse;
      }

      public double getAllDistance(){
        return drive.getAllTicks() * kEncoderDistancePerPulse;
      }
}
