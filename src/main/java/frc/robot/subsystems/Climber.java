package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    private WPI_TalonSRX winchMotor;
    private Solenoid liftSolenoid;

    private boolean deployed = false;

    private double climbMotorSpeedLimiter = 1.0;

    public Climber() {
        winchMotor = new WPI_TalonSRX(winchMotorPort);
        liftSolenoid = new Solenoid(climberSolenoidPort);
    }

    public void deploy() {
        liftSolenoid.set(true);
        deployed = true;
    }


    public void climb( Double climbSpeed) {
        winchMotor.set(climbSpeed * climbMotorSpeedLimiter);
    }

    public void withdraw() {
        liftSolenoid.set(false);
        deployed = false;
    }

    public void stop() {
        winchMotor.stopMotor();
    }

    public boolean getDeployed() {
        return deployed;
    }
}