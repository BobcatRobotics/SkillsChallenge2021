package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootShooter extends CommandBase {
    private Shooter shooter;
    private Double currtime = 0.0;
    private Double startTime = 0.0;
    private double dur = 0.0;
    private Timer t = new Timer();
    private Intake itk;
    private Feeder fdr;
    private boolean first = true;

    public ShootShooter(Shooter str , Feeder feeder, Intake intake, Double time) {
        this.shooter = str;
        this.dur = time;
        this.itk = intake;
        this.fdr = feeder;
    }

    @Override
    public void execute() {
        if(first){
            first = !first;
            this.startTime = t.getFPGATimestamp();
        }
        shooter.getToSpeed();
        itk.runLowerTowerIn(true);
        itk.runFunnelIn(true);
        fdr.run(true);
    }

    @Override
    public boolean isFinished(){
        currtime = t.getFPGATimestamp();
        if(currtime - startTime >= dur){
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interupted){
        itk.stopLowerTower();
        itk.stopFunnel();
        fdr.stop();
        shooter.stopShooter();
    }
}