package frc.robot.parts.movement;

public class MovementSettings{
    //--------------Default--------------//
    public static MovementSettings moveInLineSettings = new MovementSettings(0.15, 10, 1, 2, 0, 0);
    public static MovementSettings turnToAngleSettings = new MovementSettings(2.0, 10, 0.6, 0.1, 0, 0);

    //--------------Variables--------------//
    private double tolerance;
    private int minTimesInTolerance;
    private double maxSpeed;
    private double kp;
    private double ki;
    private double kd;
  
    MovementSettings(double tolerance, int minTimesInTolerance, double maxSpeed, double kp, double ki, double kd){
      setTolerance(tolerance);
      setMinTimesInTolerance(minTimesInTolerance);
      setMaxSpeed(maxSpeed);
      setPID(kp, ki, kd);
    }
  
    public MovementSettings setTolerance(double tolerance){
      this.tolerance = tolerance;
      return this;
    }
  
    public MovementSettings withTolerance(double tolerance){
      return new MovementSettings(tolerance, minTimesInTolerance, maxSpeed, kp, ki, kd);
    }
  
    public double getTolerance(){
      return tolerance;
    }
  
    public MovementSettings setMinTimesInTolerance(int minTimesInTolerance){
      this.minTimesInTolerance = minTimesInTolerance;
      return this;
    }
  
    public int getMinTimesInTolerance(){
      return minTimesInTolerance;
    }
  
    public MovementSettings setMaxSpeed(double maxSpeed){
      this.maxSpeed = maxSpeed;
      return this;
    }

    public MovementSettings withMaxSpeed(double maxSpeed){
      return new MovementSettings(tolerance, minTimesInTolerance, maxSpeed, kp, ki, kd);
    }
  
    public double getMaxSpeed(){
      return maxSpeed;
    }
  
    public MovementSettings setPID(double kp, double ki, double kd){
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
      return this;
    }

    public double getkP(){
        return kp;
    }

    public double getki(){
        return ki;
    }

    public double getkd(){
        return kd;
    }
  }
  
