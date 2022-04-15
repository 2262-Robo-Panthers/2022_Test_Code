package frc.robot.parts.movement;

import java.util.function.Function;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.other.PartRunner;
import frc.robot.other.Utils;
import frc.robot.parts.Part;

public class Movement extends Part{
    //--------------Variables--------------//
    private Function<Double, Double> targetSupplier;
    private Function<Movement, Double> errorSupplier;
    private Function<Double, Void> driveFunction;

    private MovementSettings settings;
    private boolean running;
    private double target;
    private int timesInTolerance;
    private PIDController PID = new PIDController(0,0,0);

    public Movement(Function<Double, Double> targetSupplier, Function<Movement, Double> errorSupplier, Function<Double, Void> driveFunction, PartRunner runner){
        super(runner);
        setTargetSupplier(targetSupplier);
        setErrorSupplier(errorSupplier);
        setDriveFunction(driveFunction);
    }

    public Movement(Function<Double, Double> targetSupplier, Function<Movement, Double> errorSupplier, Function<Double, Void> driveFunction, MovementSettings settings, PartRunner runner){
        super(runner);
        setTargetSupplier(targetSupplier);
        setErrorSupplier(errorSupplier);
        setDriveFunction(driveFunction);
        setSettings(settings);
    }

    public void init(){}

    public void setTargetSupplier(Function<Double, Double> function){
        targetSupplier = function;
    }

    public void setErrorSupplier(Function<Movement, Double> function){
        errorSupplier = function;
    }

    public void setDriveFunction(Function<Double, Void> function){
        driveFunction = function;
    }

    public void start(double target){
        this.target = targetSupplier.apply(target);
        timesInTolerance = 0;
        PID.reset();
        running = true;
    }

    public void start(double target, MovementSettings settings){
        setSettings(settings);
        start(target);
    }

    public void run(){
        if(running){
            double error = errorSupplier.apply(this);
            if(Math.abs(error) < settings.getTolerance()){
                timesInTolerance++;
                if(timesInTolerance == settings.getMinTimesInTolerance()){
                    stop();
                    return;
                }
            }else{
                timesInTolerance = 0;
            }
            driveFunction.apply(Utils.Math.capDouble(PID.calculate(error), -settings.getMaxSpeed(), settings.getMaxSpeed()));
        }
    }

    public void stop(){
        driveFunction.apply(0.0);
        running = false;
    }

    public boolean isDone(){
        return !running;
    }

    public void setSettings(MovementSettings settings){
        this.settings = settings;
        PID.setPID(settings.getkP(), settings.getki(), settings.getkd());
        PID.setIntegratorRange(-settings.getMaxSpeed(), settings.getMaxSpeed());
    }

    public MovementSettings getSettings(){
        return settings;
    }

    public double getTarget(){
        return target;
    }
}