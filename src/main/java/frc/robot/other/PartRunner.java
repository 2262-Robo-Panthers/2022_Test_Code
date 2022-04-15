package frc.robot.other;

import java.util.LinkedList;

import frc.robot.parts.Part;

public class PartRunner {
    private LinkedList<Part> parts = new LinkedList<>();

    public PartRunner(){}

    public void addPart(Part part){
        parts.add(part);
    }

    public void removeAll(){
        parts.clear();
    }

    public void initAll(){
        parts.forEach((p) -> {p.init();});
    }

    public void runAll(){
        parts.forEach((p) -> {p.run();});
    }

    public void stopAll(){
        parts.forEach((p) -> {p.stop();});
    }
}
