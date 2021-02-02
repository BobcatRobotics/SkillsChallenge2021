package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PushBot extends CommandBase {

    private boolean push;
    private Drivetrain drivetrain;
    private double speed = 1.0;
    private double currtime = 0.0;
    private double startTime = 0.0;
    private double dur = 0.0;
    private Timer t = new Timer();
    private boolean first = true;
    
    public PushBot(boolean psh, Drivetrain dt, double driveSpeed, double driveTime){
        this.push = psh;
        this.drivetrain = dt;
        this.speed = driveSpeed;
        this.dur = driveTime;
        addRequirements(drivetrain);
    }   

    @Override
    public void execute() {
        // Set percentage drive speed
        if(!push)
            return;
        if(first){
            first = !first;
            this.startTime = t.getFPGATimestamp();
        }
        drivetrain.setRightPower(speed);
        drivetrain.setLeftPower(speed);
        drivetrain.drive();
    }



    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
    @Override
    public boolean isFinished(){
        if(!push){
            return true;
        }

        currtime = t.getFPGATimestamp();
        if(currtime - startTime >= dur){
            return true;
        }
        return false;
    }

}