package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final WPI_TalonFX br = new WPI_TalonFX(3);
  private final WPI_TalonFX bl = new WPI_TalonFX(1);
  private final WPI_TalonFX fr = new WPI_TalonFX(2);
  private final WPI_TalonFX fl = new WPI_TalonFX(0);
  // private final MotorControllerGroup rightDrive = new MotorControllerGroup(fr, br);
  // private final MotorControllerGroup leftDrive = new MotorControllerGroup(fl, bl);
  //private final DifferentialDrive drive = new DifferentialDrive(leftDrive, rightDrive);
  private final DifferentialDrive drive = new DifferentialDrive(bl,br);

  private final CANSparkMax shooterLeft = new CANSparkMax(5,MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(6,MotorType.kBrushless);
  private final CANSparkMax climbTop = new CANSparkMax(7,MotorType.kBrushless);
  private final CANSparkMax climbBottom = new CANSparkMax(8,MotorType.kBrushless);
  private final CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushless);
  private final WPI_VictorSPX intakeRoller = new WPI_VictorSPX(9);

  private final Compressor PCM = new Compressor(11, PneumaticsModuleType.CTREPCM);
  private final Solenoid intake = new Solenoid(11, PneumaticsModuleType.CTREPCM, 4);
  private final Solenoid stopper = new Solenoid(11, PneumaticsModuleType.CTREPCM, 5);
  private final Solenoid shifter = new Solenoid(11, PneumaticsModuleType.CTREPCM, 6);
  private final Solenoid fanRight = new Solenoid(11,PneumaticsModuleType.CTREPCM, 7);
  private final Solenoid fanLeft = new Solenoid(11, PneumaticsModuleType.CTREPCM, 1);
  private final DoubleSolenoid climbPiston = new DoubleSolenoid(11,PneumaticsModuleType.CTREPCM,3,2);

  private final DigitalInput shootPhotoGate = new DigitalInput(0);
  private SparkMaxPIDController shooterPID = shooterLeft.getPIDController();
  private RelativeEncoder shooterEncoder = shooterRight.getEncoder(); 
 
  private final XboxController xbox = new XboxController(0);

  private boolean flywheelGoing = false;
  private double finetuning = 0.0;
  private double xboxLeftX = 0.0; 

  private static final String kDefaultAuto = "Far Auto 2 Ball";
  private static final String kCloseAuto = "Close Auto 1 Ball";
  private static final String kCloseAuto2 = "Close Auto 2 Ball";
  private static final String kCloseAuto3 = "Close Auto Defense";
  private static final String kCloseAuto4 = "Close Auto 1 Ball High";
  private static final String kCloseAuto5 = "Close Auto 4 Ball High";
  private static final String kCloseAuto6 = "Close Auto 4 Ball High Trajectory";
  private static final String kDoesNothing = "Does Nothing";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final Double k38 = 0.375;
  private static final Double k50 = 0.5;
  private static final Double k60 = 0.6;
  private static final Double k65 = 0.65;
  private static final Double k75 = 0.75;
  private static final Double k80 = 0.8;
  private static final Double k100 = 1.0;
  private final SendableChooser<Double> climbPowerSetter = new SendableChooser<>();
  private final SendableChooser<Double> flywheelPowerSetter = new SendableChooser<>();

  private double flywheelSpeed = 0;
  private final Timer autoTimer = new Timer();

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(0.0,0.0,new Rotation2d()));

  //climb
  //double minClimbPos = -1000;
  //double maxClimbPos = 970;
  //double climbSpeed = 10;
  //double climbPosTol = 2.5;
  //double climbProportional = 1;
  double maxClimbUpPower = 0.65;
  double maxClimbDownPower = -0.5;

  double maxDriveTemp = 40.0;

  // double kP = 6e-5; 
  // double kI = 0;
  // double kD = 0; 
  // double kIz = 0; 
  // double kFF = 0.000015; 
  // double kMaxOutput = 1; 
  // double kMinOutput = -1;
  // double maxRPM = 4500;


  private double dashFlySpeed;

  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(5, 2);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.43));
  private Field2d m_field = new Field2d();
  private RamseteController ramsete = new RamseteController();
  

  private int queuedShots = 0;
  private boolean isShooting = false;

  //do not edit
  //double climbSetPos = 0;

  //position tracking
<<<<<<< Updated upstream
  private double ticksPerDegree = 18.93;
  private Pose2d startPose2d = new Pose2d(0,0, Rotation2d.fromDegrees(0));
  //do not edit
  private Rotation2d angleOffset;
  private Pose2d robotPose = new Pose2d();
  private Rotation2d angle;
=======
  public static final int kEncoderCPR = 2048;
  public static final double kGearRatio = 20.8333;
  //public static final double kGearRatio = 9.167;
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public static final double kEncoderDistancePerPulse =
        ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR / (double) kGearRatio);
>>>>>>> Stashed changes


  @Override
  public void robotInit() {
    setRobotPose(startPose2d);
    
    m_chooser.setDefaultOption("Far Auto L", kDefaultAuto);
    m_chooser.addOption("Close Auto 1 Ball L", kCloseAuto);
    m_chooser.addOption("Close Auto 2 Ball L", kCloseAuto2);
    m_chooser.addOption("Close Auto Defense L", kCloseAuto3);
    m_chooser.addOption("High Auto 1 Ball H", kCloseAuto4);
    m_chooser.addOption("4 Ball Auto H", kCloseAuto5);
    m_chooser.addOption("4 Ball Auto H Trajectory", kCloseAuto6);
    m_chooser.addOption("Does Nothing", kDoesNothing);
    SmartDashboard.putData("Auto choices", m_chooser);
    climbPowerSetter.setDefaultOption("50%", k50);
    climbPowerSetter.addOption("65%", k65);
    climbPowerSetter.addOption("80%", k80);
    climbPowerSetter.addOption("100%", k100);
    SmartDashboard.putData("Climb Power", climbPowerSetter);
    flywheelPowerSetter.addOption("37.5%", k38);
    flywheelPowerSetter.addOption("60%", k60);
    flywheelPowerSetter.addOption("65%", k65);
    flywheelPowerSetter.addOption("75%", k75);
    flywheelPowerSetter.addOption("80%", k80);
    flywheelPowerSetter.setDefaultOption("100%", k100);
    SmartDashboard.putData("Flywheel Power", flywheelPowerSetter);
    fl.follow(bl);
    fr.follow(br);
    fr.setInverted(true);
    br.setInverted(true);
    shooterRight.follow(shooterLeft, true);
    fr.setNeutralMode(NeutralMode.Coast);
    fl.setNeutralMode(NeutralMode.Coast);
    bl.setNeutralMode(NeutralMode.Coast);
    br.setNeutralMode(NeutralMode.Coast);
    // shooterRight.setInverted(true);
    shooterRight.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    climbTop.setIdleMode(IdleMode.kBrake);
    climbBottom.setIdleMode(IdleMode.kBrake);
    indexer.setIdleMode(IdleMode.kBrake);
    //climbSetPos = getClimbPos();
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

    // shooterPID = shooterRight.getPIDController();
    // shooterEncoder = shooterRight.getEncoder();

    // shooterPID.setP(kP);
    // shooterPID.setI(kI);
    // shooterPID.setD(kD);
    // shooterPID.setIZone(kIz);
    // shooterPID.setFF(kFF);
    // shooterPID.setOutputRange(kMinOutput, kMaxOutput);
    
  }

  @Override
  public void robotPeriodic() {
    updateRobotPose();

    SmartDashboard.putNumber("Left Neo RPM", shooterLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Neo RPM", shooterRight.getEncoder().getVelocity());
    SmartDashboard.putNumber("FlywheelRPM", ((shooterRight.getEncoder().getVelocity()+shooterLeft.getEncoder().getVelocity())/2)*1.25);
    SmartDashboard.putNumber("FlywheelSet", shooterLeft.get());
    SmartDashboard.putBoolean("Intake Roller", intakeRoller.get()!=0);
    SmartDashboard.putBoolean("PhotoGate", shootPhotoGate.get());
    SmartDashboard.putNumber("Right Neo Temp", shooterRight.getMotorTemperature());
    SmartDashboard.putNumber("Left Neo Temp", shooterLeft.getMotorTemperature());
    SmartDashboard.putNumber("Top Climb Temp", climbTop.getMotorTemperature());
    SmartDashboard.putNumber("Bottom Climb Temp", climbBottom.getMotorTemperature());
    SmartDashboard.putNumber("FR Temp", fr.getTemperature());
    SmartDashboard.putNumber("BR Temp", br.getTemperature());
    SmartDashboard.putNumber("FL Temp", fl.getTemperature());
    SmartDashboard.putNumber("BL Temp", bl.getTemperature());
    SmartDashboard.putNumber("FL Encoder", fl.getSelectedSensorPosition());
<<<<<<< Updated upstream
    SmartDashboard.putString("robot pose", robotPose.toString());
=======
    SmartDashboard.putNumber("Fr Power", fr.get());
    SmartDashboard.putNumber("Br Power", br.get());
    SmartDashboard.putNumber("Indexer Temp", indexer.getMotorTemperature());
    SmartDashboard.putData("Field", m_field);
>>>>>>> Stashed changes

    dashFlySpeed = SmartDashboard.getNumber("fly speed", 0.6);
    SmartDashboard.putNumber("fly speed", dashFlySpeed);

    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimer.reset();
    autoTimer.start();
    shifter.set(false);
    fr.setSelectedSensorPosition(0);
    fl.setSelectedSensorPosition(0);
    intakeRoller.set(0);
    stopFlywheel();
    shifter.set(false);
    intakeRoller.set(0);
    indexer.set(0);
    var chassisSpeeds = new ChassisSpeeds(2.0, 0.0, 1.0);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;
    double linearVelocity = chassisSpeeds.vxMetersPerSecond;
    double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;
    //2,125,000 ticks per foot
    trajectoryConfig.setKinematics(kinematics);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
      default:
        // Put default auto code here
        if (autoTimer.get() < 3){
          drive.arcadeDrive(-0.65, 0);
          intake.set(true);
          intakeRoller.set(0.65);
        }
        else if (autoTimer.get() < 4){
          drive.arcadeDrive(0, 0);
          setFlywheel(0.65);
        }
        else if (autoTimer.get() < 5){
          intake.set(false);
          intakeRoller.set(0);
          if (autoTimer.get()>4.1 && autoTimer.get()<4.2){
            shoot();
          }
          if (autoTimer.get()>4.8&&autoTimer.get()<4.9){
            shoot();
          }
        }
        if (autoTimer.get() > 10){
          stopFlywheel();
        }

        break;
        case kCloseAuto:
        // Put custom auto code here
        if (autoTimer.get() < 3){
          setFlywheel(0.375);
          drive.arcadeDrive(0, 0);
        }
        else if (autoTimer.get() < 4){
          shoot();
        }
        else if (autoTimer.get() > 5){
          stopFlywheel();
          if (autoTimer.get()>5 && autoTimer.get()<10){
            intake.set(true);
            intakeRoller.set(0.65);
            drive.arcadeDrive(-0.65, 0);
          }
        }
        if (autoTimer.get() > 10){
          stopFlywheel();
          drive.arcadeDrive(0, 0);
          intakeRoller.set(0);
          intake.set(false);
        }
        break;
        case kCloseAuto2:
        // Put custom auto code here
        if (autoTimer.get() < 2){
          setFlywheel(0.385);
          drive.arcadeDrive(0, 0);
        }
        else if (autoTimer.get() < 3){
          shoot();
        }
        else if (autoTimer.get() > 3){
          if (autoTimer.get()>3.5 && autoTimer.get()<7.5){
            intake.set(true);
            intakeRoller.set(0.65);
            drive.arcadeDrive(-0.75, 0);
            stopFlywheel();
          }
        }
        if (autoTimer.get() > 7.5 && autoTimer.get() < 11.5){
          drive.arcadeDrive(0.75, 0);
          intakeRoller.set(0);
          intake.set(false);
          setFlywheel(0.4);
        }
        if (autoTimer.get()>12){
          drive.arcadeDrive(0, 0);
          setFlywheel(0.4);
        }
        if (autoTimer.get()>13){
          shoot();
        }
        break;
        case kCloseAuto3:
        // Put custom auto code here
        if (autoTimer.get() < 2){
          setFlywheel(0.385);
          drive.arcadeDrive(0, 0);
        }
        else if (autoTimer.get() < 3){
          shoot();
        }
        else if (autoTimer.get() > 3){
          if (autoTimer.get()>3.5 && autoTimer.get()<7){
            intake.set(true);
            intakeRoller.set(0.65);
            drive.arcadeDrive(-0.75, 0);
            stopFlywheel();
          }
        }
        if (autoTimer.get() > 7){
          drive.arcadeDrive(0, 0);
          intakeRoller.set(0);
          intake.set(false);
        }
        break;
        case kCloseAuto4:
          if (autoTimer.get()<2){
            setFlywheel(0.57);
          }
          else if (autoTimer.get()<4){
            shoot();
          }
          else if (autoTimer.get() < 8){
            setFlywheel(0);
            intake.set(true);
            intakeRoller.set(-0.67);
            drive.arcadeDrive(-0.75, 0);
          }
          else if (autoTimer.get()<15){
            drive.arcadeDrive(0,0);
            intakeRoller.set(0);
            intake.set(false);
          }
        break;
        case kCloseAuto5:
        // if (autoTimer.get()<3) {
        //   intake.set(true);
        //   intakeRoller.set(-0.67);
        //   drive.arcadeDrive(0.9, -0.3);
        // }
        if (autoTimer.get()<4){
          intake.set(true);
          intakeRoller.set(-0.67);
          drive.arcadeDrive(-0.8, -0.475);
        }
        else if (autoTimer.get()<5.75){
          //intake.set(false);
          //intakeRoller.set(0);
          setFlywheel(0.575);
          drive.arcadeDrive(0.9, 0.65);
        }
        else if (autoTimer.get()<6.5){
          drive.arcadeDrive(0.7, 0);
        }
        else if (autoTimer.get()<8.5){
          drive.arcadeDrive(0, 0);
          stopper.set(true);
        }
        else if (autoTimer.get()<10){
          drive.arcadeDrive(-0.7, 0);
        }
        // else if (autoTimer.get()<10) {
        //   shifter.set(true);
        //   stopFlywheel();
        //   intakeRoller.set(-0.67);
        //   drive.arcadeDrive(-0.9, -0.425);
        //   stopper.set(false);
        // }
        // else if (autoTimer.get()<11){
        //   shifter.set(false);
        //   drive.arcadeDrive(-0.5, 0);
        // }
        // else if (autoTimer.get()<11.75){
        //   drive.arcadeDrive(0, 0);
        //   shifter.set(true);
        // }
        // else if (autoTimer.get()<13.5){
        //   drive.arcadeDrive(0.9, 0.3);
        //   setFlywheel(0.55);
        // }
        // else if (autoTimer.get()<14){
        //   shifter.set(false);
        //   drive.arcadeDrive(0.7, 0);
        // }
        else {
          stopper.set(true);
          drive.arcadeDrive(0, 0);
        }
        break;
        //1 foot is 19.89 rotations and 40744 ticks
        case kCloseAuto6:
        var trajectoryOne =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0,0,Rotation2d.fromDegrees(90)),
          List.of(new Translation2d(6,6)),
          new Pose2d(3,0,Rotation2d.fromDegrees(0)),
          trajectoryConfig);
        m_field.getObject("traj").setTrajectory(trajectoryOne);
        m_field.setRobotPose(trajectoryOne.getInitialPose());
        break;
        case kDoesNothing:
        break;
      }
    }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    stopper.set(false);
    shifter.set(false);
    intake.set(false);
    climbPiston.set(Value.kForward);
    setFlywheel(0.0);
    flywheelSpeed =1.0;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    runShoot();
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    // if((p != kP)) { shooterPID.setP(p); kP = p; }
    // if((i != kI)) { shooterPID.setI(i); kI = i; }
    // if((d != kD)) { shooterPID.setD(d); kD = d; }
    // if((iz != kIz)) { shooterPID.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { shooterPID.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   shooterPID.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    //}
    maxClimbDownPower=climbPowerSetter.getSelected() * -1;
    //bl.set(-0.5);
    final int DPad = xbox.getPOV(0);

    finetuning = xbox.getRightX() * 0.3;
    if((Math.abs(finetuning)) < 0.01) finetuning = 0;

    xboxLeftX = xbox.getLeftX();
    if(Math.abs(xboxLeftX) < 0.1) xboxLeftX = 0;

    drive.arcadeDrive(xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis(), -xboxLeftX - finetuning);

    if (DPad == 0) {intake.set(false); intakeRoller.set(0);}
    if (DPad == 90){intakeRoller.set(-0.67);}
    if (DPad == 180) {intake.set(true); intakeRoller.set(0);}
    if (DPad == 270) intakeRoller.set(0.67);

    xbox.setRumble(RumbleType.kLeftRumble, shooterLeft.get());
		xbox.setRumble(RumbleType.kRightRumble, shooterRight.get());

    // if (xbox.getStartButtonPressed()) {
    //   //shooterPID.setReference(200, CANSparkMax.ControlType.kVelocity);
    //   setFlywheel(flywheelSpeed+0.025);
    //   flywheelSpeed = shooterRight.get();
    // }
    // if (xbox.getBackButtonPressed() && shooterRight.get() > -1){
    //    setFlywheel(flywheelSpeed-0.05);
    //    flywheelSpeed = shooterRight.get();
    // }
    flywheelSpeed = dashFlySpeed;
    if (xbox.getXButtonPressed()){
      if (!flywheelGoing)
        setFlywheel(flywheelSpeed);
        // shooterPID.setReference(maxRPM, CANSparkMax.ControlType.kVelocity);
      else setFlywheel(0.0);
    }

    if (xbox.getAButtonPressed()){
      newShoot();
    } else if(xbox.getRightStickButtonPressed()){
      newShoot2();
    }

    if (xbox.getBButton()){
      setClimbPower2(maxClimbDownPower);
    }
    else if (xbox.getYButton()){
      setClimbPower2(maxClimbUpPower);
    }else{
      setClimbPower2(0);
    }

    if (xbox.getRightBumperPressed()){
      shifter.set(true);
    }
    if (xbox.getLeftBumperPressed()){
      shifter.set(false);
    }
    if(xbox.getStartButtonPressed()){
      climbPiston.set(Value.kForward);
    }
    else if (xbox.getBackButtonPressed()){
      climbPiston.set(Value.kReverse);
    }

    if (fr.getTemperature()>maxDriveTemp||br.getTemperature()>maxDriveTemp){
       fanRight.set(true);
    }
    else fanRight.set(false);
    
    if (fl.getTemperature()>maxDriveTemp||bl.getTemperature()>maxDriveTemp){
       fanLeft.set(true);
    }
    // else fanLeft.set(false);
    // if (indexer.getMotorTemperature()>45||shootPhotoGate.get()&&!stopper.get()) {
    //   indexer.set(0);
    // }
    // else {indexer.set(-0.75);}

    if ((indexer.getMotorTemperature()<45&&stopper.get())||(indexer.getMotorTemperature()<45&&!shootPhotoGate.get())){
      indexer.set(-1);
    }
    else {
      indexer.set(0);
    }

    //runClimb();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  public void setFlywheel(double Percent){
    shooterLeft.set(Percent);
    //shooterRight.set(Percent);
    flywheelGoing=true;
    if (Percent == 0) flywheelGoing=false;
  }
  public void stopFlywheel(){
    shooterLeft.set(0);
    //shooterRight.set(0);
    flywheelGoing=false;
  }

  public void shoot(){
    if (shootPhotoGate.get()&&flywheelGoing){
      stopper.set(true);
      //indexer.set(-0.75);
      if (!shootPhotoGate.get()){
        stopper.set(false);
        //indexer.set(0);
      }
    }
  }

  void setClimbPower2(double power){
    if (climbBottom.getMotorTemperature() < 75 && climbTop.getMotorTemperature() < 75){
    climbTop.set(power);
    climbBottom.set(power);
    }
    else{
      climbBottom.set(0);
      climbTop.set(0);
    }
  }

  void setAngle(Rotation2d angle){
    setRobotPose(new Pose2d(robotPose.getX(), robotPose.getY(), angle));
  }

  double getAngleAsDegree(double leftDistance, double rightDistance){
    return scaleAngle(((leftDistance - rightDistance) / ticksPerDegree) + angleOffset.getDegrees());
  }

  Rotation2d getAngle(double leftDistance, double rightDistance){
    return Rotation2d.fromDegrees(getAngleAsDegree(leftDistance, rightDistance));
  }

  double scaleAngle(double rawAngle){
    rawAngle = rawAngle % 360;
    return (rawAngle < 0) ? rawAngle + 360 : rawAngle;
  }

  double getRightDriveTicks(){
    return (fr.getSelectedSensorPosition() + br.getSelectedSensorPosition()) / 2;
  }

  double getLeftDriveTicks(){
    return (fr.getSelectedSensorPosition() + br.getSelectedSensorPosition()) / 2;
  }

  void setRobotPose(Pose2d setPose){
    angle = setPose.getRotation();

    resetDriveEncoders();
    m_odometry.resetPosition(robotPose, angle);

    angleOffset = angle;
    robotPose = setPose;
  }

  void resetDriveEncoders(){
    fl.setSelectedSensorPosition(0);
    fr.setSelectedSensorPosition(0);
    bl.setSelectedSensorPosition(0);
    br.setSelectedSensorPosition(0);
  }

  void updateRobotPose(){
      double leftDrivePos = getLeftDriveTicks();
      double rightDrivePos = getRightDriveTicks();
      
      angle = getAngle(leftDrivePos, rightDrivePos);
      robotPose = m_odometry.update(angle, leftDrivePos, rightDrivePos);
  }

  void newShoot(){
    queuedShots = 1;
  }

  void newShoot2(){
    queuedShots = 2;
  }

  void runShoot(){
    if(queuedShots == 0){
      stopper.set(false);
      return;
    }

    if(!isShooting && shootPhotoGate.get()){
      if(flywheelGoing){
        stopper.set(true);
        isShooting = true;
        return;
      }
      queuedShots = 0;
      return;
    }

    if(isShooting && !shootPhotoGate.get()){
      stopper.set(false);
      isShooting = false;
      queuedShots--;
    }
  }
  // public void generateTrajectory() {
  //   var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
  //     Rotation2d.fromDegrees(-180));
  //   var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
  //     Rotation2d.fromDegrees(-160));
  // }
}