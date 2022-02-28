// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private final Solenoid stopper = new Solenoid(11, PneumaticsModuleType.CTREPCM, 6);
  private final Solenoid shifter = new Solenoid(11, PneumaticsModuleType.CTREPCM, 5);
  private final DoubleSolenoid climbPiston = new DoubleSolenoid(11,PneumaticsModuleType.CTREPCM,0,2);

  private final DigitalInput shootPhotoGate = new DigitalInput(0);
 

  private final XboxController xbox = new XboxController(0);

  private boolean flywheelGoing = false;
  private double finetuning = 0.0;
  private double xboxLeftX = 0.0; 

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private double flywheelSpeed = 0;
  private final Timer autoTimer = new Timer();

  //climb
  double minClimbPos = 0;
  double maxClimbPos = 100;
  double climbSpeed = 5;
  double climbPosTol = 5;
  double climbUpPower = 0.5;
  double climbDownPower = -0.5;
  //do not edit
  double climbSetPos = 0;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Simple Auto", kDefaultAuto);
    m_chooser.addOption("Other Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    bl.follow(fl);
    br.follow(fr);
    fr.setInverted(true);
    br.setInverted(true);
    fr.setNeutralMode(NeutralMode.Coast);
    fl.setNeutralMode(NeutralMode.Coast);
    shooterRight.setInverted(true);
    shooterRight.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    climbTop.setIdleMode(IdleMode.kBrake);
    climbBottom.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kDefaultAuto:
      default:
        // Put default auto code here
        autoTimer.reset();
        autoTimer.start();
        if (autoTimer.get() < 1){
          drive.arcadeDrive(0.75, 0);
        }
        else if (autoTimer.get() < 2){
          drive.arcadeDrive(0, 0);
        }
        else if (autoTimer.get() < 5){
          setFlywheel(1.0);
          if (autoTimer.get()>4){
            shoot();
          }
        }
        if (autoTimer.get() > 5){
          stopFlywheel();
        }

        break;
      case kCustomAuto:
        // Put custom auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    stopper.set(false);
    shifter.set(true);
    intake.set(false);
    climbPiston.set(Value.kForward);
    setFlywheel(0.0);
    flywheelSpeed =1.0;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

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

    if (xbox.getStartButtonPressed() && shooterRight.get() < 1) {
      setFlywheel(shooterRight.get()+0.025);
      flywheelSpeed = shooterRight.get();
    }
    if (xbox.getBackButtonPressed() && shooterRight.get() > -1){
      setFlywheel(shooterRight.get()-0.05);
      flywheelSpeed = shooterRight.get();
    }
    if (xbox.getXButtonPressed()){
      if (!flywheelGoing)
        setFlywheel(flywheelSpeed);
      else setFlywheel(0.0);
    }

    if (xbox.getAButtonPressed()){
      shoot();
    }

    if (xbox.getBButton()){moveClimb(-climbSpeed);}
    else if (xbox.getYButton()){moveClimb(climbSpeed);}

    if (xbox.getRightBumperPressed()){
      shifter.set(false);
      climbPiston.set(Value.kReverse);
    }
    if (xbox.getLeftBumperPressed()){
      shifter.set(true);
      climbPiston.set(Value.kForward);
    }

    if (!shootPhotoGate.get()) stopper.set(false);

    runClimb();

    SmartDashboard.putNumber("Left Neo RPM", shooterLeft.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Neo RPM", shooterRight.getEncoder().getVelocity());
    SmartDashboard.putNumber("FlywheelRPM", ((shooterRight.getEncoder().getVelocity()+shooterLeft.getEncoder().getVelocity())/2)*1.25);
    SmartDashboard.putNumber("FlywheelSet", shooterLeft.get());
    SmartDashboard.putBoolean("Intake Roller", intakeRoller.get()!=0);
    SmartDashboard.putBoolean("PhotoGate", shootPhotoGate.get());
    SmartDashboard.putNumber("Right Neo Temp", shooterRight.getMotorTemperature());
    SmartDashboard.putNumber("Left Neo Temp", shooterLeft.getMotorTemperature());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  public void setFlywheel(double Percent){
    shooterLeft.set(Percent);
    shooterRight.set(Percent);
    flywheelGoing=true;
    if (Percent == 0) flywheelGoing=false;
  }
  public void stopFlywheel(){
    shooterLeft.set(0);
    shooterRight.set(0);
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
  public void setClimb(double pos){
    climbSetPos = Math.max(minClimbPos, Math.min(maxClimbPos, pos));
  }
  public void moveClimb(double amount){
    setClimb(climbSetPos + amount);
  }
  void runClimb(){
    double currPos = climbTop.getEncoder().getPosition();
    if(currPos > climbSetPos + climbPosTol){
      climbTop.set(climbDownPower);
      climbBottom.set(climbDownPower);
    }else if(currPos < climbSetPos - climbPosTol){
      climbTop.set(climbUpPower);
      climbBottom.set(climbUpPower);
    }
    else{
      climbTop.set(0);
      climbBottom.set(0);
    }
  }
}

