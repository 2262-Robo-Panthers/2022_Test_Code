// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final WPI_TalonFX br = new WPI_TalonFX(3);
  private final WPI_TalonFX bl = new WPI_TalonFX(1);
  private final WPI_TalonFX fr = new WPI_TalonFX(2);
  private final WPI_TalonFX fl = new WPI_TalonFX(0);
  private final DifferentialDrive drive = new DifferentialDrive(fl, fr);

  private final CANSparkMax shooterLeft = new CANSparkMax(5,MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(6,MotorType.kBrushless);
  private final CANSparkMax climbTop = new CANSparkMax(7,MotorType.kBrushless);
  private final CANSparkMax climbBottom = new CANSparkMax(8,MotorType.kBrushless);
  private final WPI_VictorSPX intakeRoller = new WPI_VictorSPX(9);

  private final Compressor PCM = new Compressor(11, PneumaticsModuleType.CTREPCM);
  private final Solenoid intake = new Solenoid(11, PneumaticsModuleType.CTREPCM, 4);
  private final Solenoid stopper = new Solenoid(11, PneumaticsModuleType.CTREPCM, 5);
  private final Solenoid shifter = new Solenoid(11, PneumaticsModuleType.CTREPCM, 6);
  private final DoubleSolenoid climbPiston = new DoubleSolenoid(11,PneumaticsModuleType.CTREPCM,3,2);

  private final DigitalInput shootPhotoGate = new DigitalInput(0);
 
  private final XboxController xbox = new XboxController(0);

  private boolean flywheelGoing = false;
  private double finetuning = 0.0;
  private double xboxLeftX = 0.0; 

  private static final String kDefaultAuto = "Far Auto 2 Ball";
  private static final String kCloseAuto = "Close Auto 1 Ball";
  private static final String kCloseAuto2 = "Close Auto 2 Ball";
  private static final String kCloseAuto3 = "Close Auto Defense";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final Double k38 = 0.375;
  private static final Double k50 = 0.5;
  private static final Double k65 = 0.65;
  private static final Double k75 = 0.75;
  private static final Double k80 = 0.8;
  private static final Double k100 = 1.0;
  private final SendableChooser<Double> climbPowerSetter = new SendableChooser<>();
  private final SendableChooser<Integer> flywheelRPMSetter = new SendableChooser<>();

  private final Timer autoTimer = new Timer();

  //climb
  //double minClimbPos = -1000;
  //double maxClimbPos = 970;
  //double climbSpeed = 10;
  //double climbPosTol = 2.5;
  //double climbProportional = 1;
  double maxClimbUpPower = 0.65;
  double maxClimbDownPower = -0.5;

  //----------launch----------//
  //PID
  private PIDController shooterPID = new PIDController(0, 0, 0);
   double kP = 6e-5; 
   double kI = 0;
   double kD = 0; 
   double kIz = 1; 
   //double kFF = 0.000015; 
   double kMaxOutput = 1; 
   double kMinOutput = 0;
  //other
  double LaunchMinRPM = 0;
  double LaunchMaxRPM = 7500;
  double LaunchRPM = 0;

  //do not edit
  //double climbSetPos = 0;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Far Auto", kDefaultAuto);
    m_chooser.addOption("Close Auto 1 Ball", kCloseAuto);
    m_chooser.addOption("Close Auto 2 Ball", kCloseAuto2);
    m_chooser.addOption("Close Auto Defense", kCloseAuto3);
    SmartDashboard.putData("Auto choices", m_chooser);
    climbPowerSetter.setDefaultOption("50%", k50);
    climbPowerSetter.addOption("65%", k65);
    climbPowerSetter.addOption("80%", k80);
    climbPowerSetter.addOption("100%", k100);
    SmartDashboard.putData("Climb Power", climbPowerSetter);
    flywheelRPMSetter.addOption("1000 PRM", 1000);
    flywheelPowerSetter.addOption("65%", k65);
    flywheelPowerSetter.addOption("75%", k75);
    flywheelPowerSetter.addOption("80%", k80);
    flywheelPowerSetter.setDefaultOption("100%", k100);
    SmartDashboard.putData("Flywheel Set RPM", RPM);

    bl.follow(fl);
    br.follow(fr);
    fr.setInverted(true);
    br.setInverted(true);
    shooterRight.follow(shooterLeft, true);
    fr.setNeutralMode(NeutralMode.Brake);
    fl.setNeutralMode(NeutralMode.Brake);
    bl.setNeutralMode(NeutralMode.Brake);
    br.setNeutralMode(NeutralMode.Brake);
    // shooterRight.setInverted(true);
    shooterRight.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    climbTop.setIdleMode(IdleMode.kBrake);
    climbBottom.setIdleMode(IdleMode.kBrake);
    //climbSetPos = getClimbPos();
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);

     shooterPID.setP(kP);
     shooterPID.setI(kI);
     shooterPID.setD(kD);
    //shooterPID.setIZone(kIz);
    // shooterPID.setFF(kFF);
     shooterPID.setIntegratorRange(0, kIz);
    
  }

  @Override
  public void robotPeriodic() {
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
     double p = SmartDashboard.getNumber("P Gain", 0);
     double i = SmartDashboard.getNumber("I Gain", 0);
     double d = SmartDashboard.getNumber("D Gain", 0);
     double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
     double max = SmartDashboard.getNumber("Max Output", 0);
     double min = SmartDashboard.getNumber("Min Output", 0);

     if((p != kP)) { shooterPID.setP(p); kP = p; }
     if((i != kI)) { shooterPID.setI(i); kI = i; }
     if((d != kD)) { shooterPID.setD(d); kD = d; }
     if((iz != kIz)) { shooterPID.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { shooterPID.setFF(ff); kFF = ff; }
     if((max != kMaxOutput) || (min != kMinOutput)) {  
       kMinOutput = min; kMaxOutput = max; 
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
    if (DPad == 90){
      if (intakeRoller.get()<=0) {
        intakeRoller.set(0.67);
      }
      else intakeRoller.set(-0.67);
    }
    if (DPad == 180) intake.set(true);
    if (DPad == 270) intakeRoller.set(0);

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
    flywheelSpeed = flywheelPowerSetter.getSelected();
    if (xbox.getXButtonPressed()){
      if (!flywheelGoing)
        setFlywheel(flywheelSpeed);
        // shooterPID.setReference(maxRPM, CANSparkMax.ControlType.kVelocity);
      else setFlywheel(0.0);
    }

    if (xbox.getAButtonPressed()){
      shoot();
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
      climbPiston.set(Value.kForward);
    }
    if (xbox.getRightStickButtonPressed()){
      climbPiston.set(Value.kReverse);
    }

    if (!shootPhotoGate.get()) stopper.set(false);

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
  /*
  public void climbUp(){
    climbTop.set(0.5);
    climbBottom.set(0.5);
  }
  public void climbDown(){
    climbTop.set(-1);
    climbBottom.set(-1);
  }
  public void climbStop(){
    climbTop.set(0);
    climbBottom.set(0);
  }
  */
  public void shoot(){
    if (shootPhotoGate.get()&&flywheelGoing){
      stopper.set(true);
      if (!shootPhotoGate.get()){
        stopper.set(false);
      }
    }
  }
  /*
  void setClimb(double pos){
    climbSetPos = Math.max(minClimbPos, Math.min(maxClimbPos, pos));
  }
  void moveClimb(double amount){
    setClimb(getClimbPos() + amount);
  }
  double getClimbPos(){
    return (climbTop.getEncoder().getPosition() + climbBottom.getEncoder().getPosition()) / 2.0;
  }
  void setClimbPower(double power){
    double pow = Math.max(maxClimbDownPower, Math.min(maxClimbUpPower, power));
    climbTop.set(pow);
    climbBottom.set(pow);
  }
  void runClimb(){
    double error = climbSetPos - getClimbPos();
    if(Math.abs(error) > climbPosTol){ 
      setClimbPower(error * climbProportional);
    }else{
      setClimbPower(0);
    }
  }
  */

  void setClimbPower2(double power){
    if (climbBottom.getMotorTemperature() < 40 && climbTop.getMotorTemperature() < 40){
    climbTop.set(power);
    climbBottom.set(power);
    }
    else{
      climbBottom.set(0);
      climbTop.set(0);
    }
  }
}

