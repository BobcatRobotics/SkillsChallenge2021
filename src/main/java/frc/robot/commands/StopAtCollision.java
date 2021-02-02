package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.lib.RioLogger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavxGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// If a collision is detected in auto mode, then stop the robot for 1 second
public class StopAtCollision extends CommandBase {
    private NavxGyro gyro = new NavxGyro(SPI.Port.kMXP);
    private Drivetrain dt = new Drivetrain();
    double lastWorldAccelX = 0.0;
    double lastWorldAccelY = 0.0;
    double lastWorldAccelZ = 0.0;
    final static double kCollisionThreshold_DeltaG = 1.2; // Jerk (m/s^3) threshold
    public static boolean collisionDetected = false;

    public StopAtCollision(NavxGyro gyro, Drivetrain dt) {
        this.dt = dt;
        this.gyro = gyro;
        gyro.calibrate();
    }

    public void DetectCollision() {
        // Calculating jerk
        double currWorldAccelX = gyro.getWorldLinearAccelX();
        double currentJerkX = currWorldAccelX - lastWorldAccelX;
        lastWorldAccelX = currWorldAccelX;

        double currWorldAccelY = gyro.getWorldLinearAccelY();
        double currentJerkY = currWorldAccelY - lastWorldAccelY;
        lastWorldAccelY = currWorldAccelX;

        double currWorldAccelZ = gyro.getWorldLinearAccelZ();
        double currentJerkZ = currWorldAccelZ - lastWorldAccelZ;
        lastWorldAccelZ = currWorldAccelZ;

        SmartDashboard.putNumber("Acceleration X", currWorldAccelX);
        SmartDashboard.putNumber("Acceleration Y", currWorldAccelY);
        SmartDashboard.putNumber("Acceleration Z", currWorldAccelZ);
        SmartDashboard.putNumber("Jerk X", currentJerkX);
        SmartDashboard.putNumber("Jerk Y", currentJerkY);
        SmartDashboard.putNumber("Jerk Z", currentJerkZ);

        // Testing the actual jerk against the threshold
        if (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG
                || Math.abs(currentJerkX) > kCollisionThreshold_DeltaG
                && Math.abs(currentJerkZ) > kCollisionThreshold_DeltaG) {

            Constants.StopAtCollisionConstants.collision = true;
        } else {
            Constants.StopAtCollisionConstants.collision = false;
        }
    }

    public void StopIfCollision(boolean collisionDetected) throws InterruptedException {
        if (collisionDetected) {
            //TODO: Make this actually do something useful
            // Stop the drivetrain
            dt.stop();
            SmartDashboard.putBoolean("Collision? ", true);
        } else {
            SmartDashboard.putBoolean("Collision? ", false);
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        DetectCollision();
        try {
            StopIfCollision(collisionDetected);
        } catch (InterruptedException e) {
            RioLogger.log("Interrupted Exception " + e);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            isFinished();
        }
    }

    @Override 
    public boolean isFinished() {
        return true;
    }
}