package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase{

    private double waitTime;
    private Timer t = new Timer();
    private boolean fpass = true;
    private double startTime;
    private double currTime;
    public Wait(Double t){
        this.waitTime = t;
    }

    @Override
    public void execute(){
        if(fpass){
            fpass = false;
            startTime = t.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished(){
        currTime = t.getFPGATimestamp();
        if((currTime - startTime) > waitTime){
            return true;
        }
        return false;

    }
}