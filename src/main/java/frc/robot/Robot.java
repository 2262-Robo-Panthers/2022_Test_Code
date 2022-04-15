package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.other.PartRunner;
import frc.robot.other.Utils;
import frc.robot.parts.Drive;
import frc.robot.parts.Position;
import frc.robot.parts.movement.Movement;
import frc.robot.parts.movement.MovementSettings;


public class Robot extends TimedRobot {

  private final XboxController xbox = new XboxController(0);
  private final PartRunner runner = new PartRunner();

  //------------Drive------------//
  Drive drive = new Drive(xbox, runner);

  //------------Position------------//
  Position position = new Position(drive, runner);

  //------------Movement------------//
  Movement moveInLine = new Movement(
    (target) -> (target + position.getAllDistance()),
    (m) -> (m.getTarget() - position.getAllDistance()), 
    (power) -> {
      drive.setPower(power, drive.getZPower());
      return null;
    }, MovementSettings.moveInLineSettings, runner);

  Movement turnToAngle = new Movement(
    (target) -> (target),
    (m) -> (Utils.Math.findAngleError(position.getAngle().getDegrees(), m.getTarget())),
    (power) -> {
      drive.setPower(drive.getXPower(), power);
      return null;
    }, MovementSettings.turnToAngleSettings, runner);


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
  private final DoubleSolenoid climbPiston = new DoubleSolenoid(11,PneumaticsModuleType.CTREPCM,3,2);

  private final DigitalInput shootPhotoGate = new DigitalInput(0);

  private boolean flywheelGoing = false; 

  private static final String kDefaultAuto = "Default Auto";
  private static final String kDoesNothing = "Does Nothing";
  private static final String k3BallAuto = "3 Ball Auto";
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

  //climb
  double maxClimbUpPower = 0.65;
  double maxClimbDownPower = -0.5;


  private double dashFlySpeed;

  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(5, 2);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.43));
  private Field2d m_field = new Field2d();
  

  private int queuedShots = 0;
  private boolean isShooting = false;

  //auto
  private static int autoStep = 0;

  @Override
  public void robotInit() {
    runner.initAll();

    m_chooser.setDefaultOption("3 Ball Auto", k3BallAuto);
    m_chooser.addOption("Far Auto L", kDefaultAuto);
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
   
    shooterRight.follow(shooterLeft, true);
    
    shooterRight.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    climbTop.setIdleMode(IdleMode.kBrake);
    climbBottom.setIdleMode(IdleMode.kBrake);
    indexer.setIdleMode(IdleMode.kBrake);
    
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    
  }

  @Override
  public void robotPeriodic() {
    runner.runAll();
    runShoot();
    runIndexer();

    m_field.setRobotPose(position.getPose());

    SmartDashboard.putNumber("Left Neo RPM", shooterLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Neo RPM", shooterRight.getEncoder().getVelocity());
    SmartDashboard.putNumber("FlywheelRPM", ((shooterRight.getEncoder().getVelocity()+shooterLeft.getEncoder().getVelocity())/2)*1.25);
    SmartDashboard.putBoolean("PhotoGate", shootPhotoGate.get());
    SmartDashboard.putNumber("Right Neo Temp", shooterRight.getMotorTemperature());
    SmartDashboard.putNumber("Left Neo Temp", shooterLeft.getMotorTemperature());
    SmartDashboard.putNumber("Top Climb Temp", climbTop.getMotorTemperature());
    SmartDashboard.putNumber("Bottom Climb Temp", climbBottom.getMotorTemperature());

    SmartDashboard.putNumber("Indexer Temp", indexer.getMotorTemperature());
    SmartDashboard.putNumber("Gyro", position.getAngle().getDegrees());
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("auto step", autoStep);
    SmartDashboard.putNumber("all distance", position.getAllDistance());

    dashFlySpeed = SmartDashboard.getNumber("fly speed", 0.6);
    SmartDashboard.putNumber("fly speed", dashFlySpeed);
  }

  @Override
  public void autonomousInit() {
    position.resetAngle();
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimer.reset();
    autoTimer.start();
    shifter.set(false);
    intakeRoller.set(0);
    stopFlywheel();
    shifter.set(false);
    intakeRoller.set(0);
    indexer.set(0);
    trajectoryConfig.setKinematics(kinematics);

    drive.resetEncoders();
    autoStep = 0;
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
        default:
        // Put default auto code here
        if (autoTimer.get() < 3){
          drive.setPower(-0.65, 0);
          intake.set(true);
          intakeRoller.set(0.65);
        }
        else if (autoTimer.get() < 4){
          drive.stop();
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
        
        case kDoesNothing:
          break;
        
        case k3BallAuto:
            if(autoStep == 0){
              setFlywheel(0.6);
              intake.set(true);
              moveInLine.start(-0.8);
              autoStep++;
            }
            else if(autoStep == 1){
              if(moveInLine.isDone())
                autoStep++;  
            }
            else if(autoStep == 2){
              newShoot();
              autoStep++;
            }
            else if(autoStep == 3){ 
              if(queuedShots == 0){
                setFlywheel(0);
                autoStep++;
              }
            }
            else if(autoStep == 4){
              turnToAngle.start(-58);
              autoStep++;
            }
            else if(autoStep == 5){
              if(turnToAngle.isDone())
                autoStep++;
            }
            else if(autoStep == 6){
              intakeRoller.set(-0.67);
              moveInLine.start(-6);
              autoStep++;
            }
            else if(autoStep == 7){
              if(moveInLine.isDone())
                autoStep++;
            }
            else if(autoStep == 8){
              intakeRoller.set(0);
              setFlywheel(0.6);
              moveInLine.start(5.7);
              autoStep++;
            }
            else if(autoStep == 9){
              if(moveInLine.isDone())
                autoStep++;
            }
            else if(autoStep == 10){
              turnToAngle.start(0);
              autoStep++;
            }
            else if(autoStep == 11){
              if(turnToAngle.isDone())
                autoStep++;
            }
            else if(autoStep == 12){
              newShoot2();
              autoStep++;
            }
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
    
    maxClimbDownPower=climbPowerSetter.getSelected() * -1;
    //bl.set(-0.5);
    final int DPad = xbox.getPOV(0);

    if (DPad == 0) {intake.set(false); intakeRoller.set(0);}
    if (DPad == 90){intakeRoller.set(-0.67);}
    if (DPad == 180) {intake.set(true); intakeRoller.set(0);}
    if (DPad == 270) intakeRoller.set(0.67);

    xbox.setRumble(RumbleType.kLeftRumble, shooterLeft.get());
		xbox.setRumble(RumbleType.kRightRumble, shooterRight.get());

    
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

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    runner.stopAll();
  }

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

  public void runIndexer(){
    if ((indexer.getMotorTemperature()<45&&stopper.get())||(indexer.getMotorTemperature()<45&&!shootPhotoGate.get())){
      indexer.set(-1);
    }
    else {
    indexer.set(0);
    }

  }
}