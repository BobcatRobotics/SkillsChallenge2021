package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {

    private final Shooter shooter;
    private final Feeder feeder;
    private final Limelight limelight;
    private final Joystick gamepad;
    private double dist = 0.0;

    // How high the outer port is above the ground (inches)
    private final double targetHeight = 98.25;
    // How high the shooter is above the ground (inches)
    private final double mountHeight = 36.0;
    // in radians, equivalent to 30 degrees
    private final double angleToGround = (Math.PI / 6.0);
    private double angleToTarget = 0.0;

    private double ty;
    private double tx;

    /**
     * Runs the shooter
     * 
     * @param sht Shooter subsystem
     * @param fd Feeder subsystem
     * @param ll Limelight subsystem
     */
    public RunShooter(Shooter sht, Feeder fd, Limelight ll, Joystick gpd) {
        this.shooter = sht;
        this.feeder = fd;
        this.limelight = ll;
        this.gamepad = gpd;
        addRequirements(shooter, feeder);
    }

    @Override
    public void execute() {
        // updateLimelightTracking();
        // dist = findDistance();
        if(shooter.isRunning())
        {
            shooter.stopShooter();
            shooter.setRunning(false);
        }else{
            shooter.setRunning(true);
            while (!shooter.atSpeed()) {
                shooter.getToSpeed();
            }
        }

        
        /*
        if (shooter.voltageSpike()) {
            dataRecorder.setDistance(dist);
            dataRecorder.setX(limelight.x());
            dataRecorder.setY(limelight.y());
        } */
        
    }

    /**
     * @return the distance to the target
     */
    public double findDistance() {
        angleToTarget = ty; // Get from limelight
        return ((targetHeight - mountHeight) / (double) (Math.tan(angleToGround + angleToTarget)));
    }

    /**
     * Checks if the limelight has a target.
     * If it does, update the tx and ty values.
     */
    public void updateLimelightTracking() {
        if (limelight.hasTarget()) {
            tx = limelight.x();
            ty = limelight.y();
        } else {
            dist = 0.0;
        }
    }

    /**
     * Command stops when interrupted
     */
    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}