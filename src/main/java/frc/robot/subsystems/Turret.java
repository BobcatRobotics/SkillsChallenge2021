package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class Turret extends SubsystemBase {
    private final WPI_TalonSRX turretMotor;

    private double spinPower;
    private Solenoid turretSolenoid;
    private boolean folded = true;
    private Limelight limelight;

    private double tx = 0.0;
    private double ty= 0.0;
    private double thresh=0.5;
    private double turretKp = 0.1; // Initial guess, 0.03 ~ 1.0PctOutput/30degrees
    private double minTurretCmd = 0.06;  // Min cmd to make turret move at all
    private boolean hasTarget = false;
    private double turretCmd = 0.0;

    public Turret(Limelight llt) {
        turretMotor = new WPI_TalonSRX(turretMotorPort);
        // When the motor is in neutral mode the motor will keep moving easily (coast)
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretSolenoid = new Solenoid(turretSolenoidPort);
        turretMotor.configFactoryDefault();
        turretMotor.setInverted(false);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        turretMotor.setSelectedSensorPosition(0,0,0);
        turretMotor.setSensorPhase(false);
        this.limelight = llt;
    }
    /**
     * Sets the spin power of the turret. This makes sure that it is
     * not less than -1 or greater than 1.
     */
    public void setSpinPower(double spinPwr) {
        if (spinPwr > 1.0) {
            spinPwr = 1.0;
        } else if (spinPwr < -1.0) {
            spinPwr = -1.0;
        }
        this.spinPower = spinPwr;
        //If we are below 
        
        //if (turretMotor.getStatorCurrent() < turretMotorHardStopCurrentThreshold) {
            turretMotor.set(spinPower);
        // }
    }

    /**
     * Retract turret
     */
    public void foldTurret() {
        turretSolenoid.set(false);
        folded = true;
    }

    public void targetEntity() {
        limelight.setCAM(0);
        limelight.setLED(3);
        hasTarget = limelight.hasTarget();
        turretCmd = 0.0;
        if (hasTarget) {
          tx = limelight.x();
          ty = limelight.y();
        } else {
          tx = 0.0;
          ty = 0.0;
        }
        // if tx is between -0.5 and +0.5 then don't move, otherwise try and get there
        if ((tx < -thresh) || (tx > thresh)) {
          // If tx is positive the target is to the right in the limelight frame, and need to
          // turn to the right to bring target toward center,
          // on the skatebot this is a negative command to the turret motor control
          if (tx > thresh) {
              if (getPosition() >= -10650) {
                //if (!turretAtHardStop()) {
                turretCmd = 1.0*turretKp*(tx-thresh) - minTurretCmd;
              }
          }
          if (tx < -thresh) {
              if (getPosition() <= 10650) {
                //if (!turretAtHardStop()) {
                turretCmd = 1.0*turretKp*(tx+thresh) + minTurretCmd;
              }
          }
        }

        //Send the desired speed
        setSpinPower(turretCmd);
    }

    /**
     * Expand turret
     */
    public void unfoldTurret() {
        turretSolenoid.set(true);
        folded = false;
    }

    /**
     * Check if turret is folded down
     */
    public boolean getFolded() {
        return folded;
    }

    /**
     * Get motor at hardstop
     */
    public boolean turretAtHardStop() {
        return turretMotor.getStatorCurrent() >= turretMotorHardStopCurrentThreshold;
    }

    /**
     * Stops the turret motor.
     */
    public void stopTurret() {
        spinPower = 0.0;  
        turretMotor.stopMotor();
    }

    public double getPosition(){
        return turretMotor.getSelectedSensorPosition();
    }

    public void zeroTurret(){
        turretMotor.setSelectedSensorPosition(0,0,0);
    }

    public WPI_TalonSRX getTurretMotor() {
        return turretMotor;
    }
}