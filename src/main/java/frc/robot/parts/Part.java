package frc.robot.parts;

import frc.robot.other.PartRunner;

public abstract class Part {
    public Part(PartRunner runner){
        runner.addPart(this);
    }

    public abstract void init();

    public abstract void run();

    public abstract void stop();
}
