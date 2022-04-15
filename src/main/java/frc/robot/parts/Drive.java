package frc.robot.parts;

import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.other.PartRunner;
import frc.robot.other.Utils;

public class Drive extends Part{
    //--------------Settings--------------//
    //input
    public Function<XboxController, Double> xPowerSupplier = (c) -> {
        return 0.0;
    };
    public Function<XboxController, Double> zPowerSupplier = (c) -> {
        return 0.0;
    };
    public double xSpeedMutiplier = 1;
    public double zSpeedMutiplier = 1;
    //motor nums
    public int flNum = 0;
    public int blNum = 1;
    public int frNum = 2;
    public int brNum = 3;
    //flip motors
    public boolean flipFL = false;
    public boolean flipBL = false;
    public boolean flipFR = true;
    public boolean flipBR = true;
    //nutral modes
    public NeutralMode flNeutralMode = NeutralMode.Coast;
    public NeutralMode blNeutralMode = NeutralMode.Coast;
    public NeutralMode frNeutralMode = NeutralMode.Coast;
    public NeutralMode brNeutralMode = NeutralMode.Coast;
    //ramp
    public boolean rampForTeleOp = false;
    public double maxXRamp = 0.07;
    public double maxZRamp = 0.07;
    //speed
    public double maxXSpeed = 1;
    public double maxZSpeed = 1;
    
    //--------------Variables--------------//
    public WPI_TalonFX br;
    public WPI_TalonFX bl;
    public WPI_TalonFX fr;
    public WPI_TalonFX fl;

    private DifferentialDrive drive;
    private XboxController controller;

    private boolean powerUpdated;
    private double xPower;
    private double zPower;

    ///////
    //set//
    ///////
    //public Drive(){
    //    init();
    //}

    public Drive(XboxController controller, PartRunner runner){
        super(runner);
        setController(controller);
    }

    public void init(){
        makeMotors();
        setMotors();

        //set drive
        drive = new DifferentialDrive(bl,br);
    }

    public void makeMotors(){
        //make motors
        br = new WPI_TalonFX(brNum);
        bl = new WPI_TalonFX(blNum);
        fr = new WPI_TalonFX(frNum);
        fl = new WPI_TalonFX(flNum);
    }

    public void setMotors(){
        //flip motors
        br.setInverted(flipBR);
        bl.setInverted(flipBL);
        fr.setInverted(flipFR);
        fl.setInverted(flipFL);
        //set cost
        br.setNeutralMode(brNeutralMode);
        bl.setNeutralMode(blNeutralMode);
        fr.setNeutralMode(frNeutralMode);
        fl.setNeutralMode(flNeutralMode);
        //follow motors
        fl.follow(bl);
        fr.follow(br);

    }

    public void setController(XboxController controller){
        this.controller = controller;
    }

    public void setSpeedmultiplier(double x, double z){
        xSpeedMutiplier = x;
        zSpeedMutiplier = z;
    }

    public void setMaxSpeed(double x, double z){
        maxXSpeed = x;
        maxZSpeed = z;
    }

    /////////
    //power//
    /////////
    public void setPower(double x, double z){
        //process variables
        x = Utils.Math.capDouble(x, -maxXSpeed, maxXSpeed);
        z = Utils.Math.capDouble(z, -maxZSpeed, maxZSpeed);

        //claculate x ramp
        double xDiff = x - xPower;
        if(xDiff > maxXRamp)
            xPower += maxXRamp;
        else if(xDiff < -maxXRamp)
            xPower -= maxXRamp;
        else
            xPower = x;

        //calculate z ramp
        double zDiff = z - zPower;
        if(zDiff > maxZRamp)
            zPower += maxZRamp;
        else if(zDiff < -maxZRamp)
            zPower -= maxZRamp;
        else
            zPower = z;

        powerUpdated = true;
    }
    public void setPowerUnramped(double x, double z){
        xPower = Utils.Math.capDouble(x, -maxXSpeed, maxXSpeed);
        zPower = Utils.Math.capDouble(z, -maxZSpeed, maxZSpeed);

        powerUpdated = true;
    }

    public void stop(){
        xPower = 0;
        zPower = 0;
        drive.arcadeDrive(0, 0);
    }

    public void run(){
        if(!powerUpdated)
            if(rampForTeleOp)
                setPower(xPowerSupplier.apply(controller) * xSpeedMutiplier, zPowerSupplier.apply(controller) * zSpeedMutiplier);
            else
                setPowerUnramped(xPowerSupplier.apply(controller) * xSpeedMutiplier, zPowerSupplier.apply(controller) * zSpeedMutiplier);

        drive.arcadeDrive(xPower, zPower);
        powerUpdated = false;
    }

    public double getXPower(){
        return xPower;
    }

    public double getZPower(){
        return zPower;
    };

    /////////
    //Ticks//
    /////////
    public double getRightTicks(){
        return (fr.getSelectedSensorPosition() + br.getSelectedSensorPosition()) / 2;
      }
    
    public double getLeftTicks(){
        return (fr.getSelectedSensorPosition() + br.getSelectedSensorPosition()) / 2;
      }

    public void resetEncoders(){
        fl.setSelectedSensorPosition(0);
        fr.setSelectedSensorPosition(0);
        bl.setSelectedSensorPosition(0);
        br.setSelectedSensorPosition(0);
      }

    public double getAllTicks(){
        return (fr.getSelectedSensorPosition() + br.getSelectedSensorPosition() + fl.getSelectedSensorPosition() + bl.getSelectedSensorPosition()) / 4.0;
      }
}
