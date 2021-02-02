package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends CommandBase {
    public Shooter shooter;
    public SpinUpShooter(Shooter str){
        shooter = str;
    }

    @Override
    public void execute() {
        shooter.getToSpeed();
    }

    @Override
    public boolean isFinished(){
        if(shooter.atSpeed()){
            return true;
        }
        else{
            return false;
        }
    }
}