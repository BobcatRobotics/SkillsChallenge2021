package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final double degreesOfTurret = 23;
    private final double diameterOfTurretWheelInches = 4;

    private final WPI_TalonFX shooterFalcon1;
    private final WPI_TalonFX shooterFalcon2;

    private final double rightRatio = 1.0;
    private final double leftRatio = 1.0;
    private final double encoderEPR = 2048.0;

    private Solenoid pancakeSolenoid;

    private double rightRPM;
    private double leftRPM;
    private boolean pancakeExpanded = true;

    //TODO: Tune this value
    private final double rpmThreshold = 400.0; 

    private final double voltThreshold = 1.0;
    
    private boolean isRunning = false;

    private double speed = 4800.0;

    /**
     * Shoots the balls into the targets.
     */
    public Shooter() {

        shooterFalcon1 = new WPI_TalonFX(shooterFalcon1Port);
        shooterFalcon2 = new WPI_TalonFX(shooterFalcon2Port);
        
        shooterFalcon1.configFactoryDefault();
        shooterFalcon2.configFactoryDefault();
        // When the motors are in neutral mode the motors will keep moving easily (coast)
        shooterFalcon1.setNeutralMode(NeutralMode.Coast);
        shooterFalcon2.setNeutralMode(NeutralMode.Coast);

        // This means the left motor speed will be equal to the right motor speed
        //shooterFalcon2.follow(shooterFalcon1);
        // The left motor needs to spin the opposite direction of the right motor
        shooterFalcon1.setInverted(true);
        //shooterFalcon2.setInverted(InvertType.OpposeMaster);
        shooterFalcon2.setInverted(false);

        shooterFalcon1.configVoltageCompSaturation(12);
        shooterFalcon1.enableVoltageCompensation(false);
        shooterFalcon2.configVoltageCompSaturation(12);
        shooterFalcon2.enableVoltageCompensation(false);

        // Setup sensors, set current positoin to 0.0 and choose phase if needed.
        shooterFalcon1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        shooterFalcon1.setSelectedSensorPosition(0,0,0);
        shooterFalcon1.setSensorPhase(false);
        shooterFalcon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        shooterFalcon2.setSelectedSensorPosition(0,0,0);
        shooterFalcon2.setSensorPhase(false);

        shooterFalcon1.config_kF(0,0.047,0);
        shooterFalcon1.config_kP(0,0.0015,0);
        shooterFalcon1.config_kI(0,0.00002,0);
        shooterFalcon1.config_kD(0,0.0,0);
        shooterFalcon1.config_IntegralZone(0,3,0);
        shooterFalcon2.config_kF(0,0.047,0);
        shooterFalcon2.config_kP(0,0.0015,0);
        shooterFalcon2.config_kI(0,0.00002,0);
        shooterFalcon2.config_kD(0,0.0,0);
        shooterFalcon2.config_IntegralZone(0,3,0);

        pancakeSolenoid = new Solenoid(shooterAngleSolenoid);
    }

    public boolean getPancakeExpanded() {
        return pancakeExpanded;
    }

    /**
     * Set whether the pancake is expanded or not
     */
    public void setPancake(boolean expanded) {
        pancakeSolenoid.set(expanded);
        pancakeExpanded = expanded;
    }

    /**
     * Starts the shooter motors
     */
    public void getToSpeed() {
        shooterFalcon1.set(ControlMode.Velocity,speed/600.0 * 2048.0);
        shooterFalcon2.set(ControlMode.Velocity,speed/600.0 * 2048.0);
    }

    /**
     * Reverse shooter motors
     */
    public void reverseShooters() {
        //Is this 1%?
        shooterFalcon1.set(-.01);
    }

    /**
     * @param distance in ft
     * @return Required rpm of turret motor to shoot that far
     */
    public int DistanceToRPM(double distance){
        distance = distance / 2;
        double val = distance * 32.6;
        val = val / (Math.sin(2 * degreesOfTurret));
        val = Math.sqrt(val);
        return (int)(val / (Math.PI * (diameterOfTurretWheelInches / 12))) * 60;
    }

    /**
     * @return Updates the RPM value of the right shooter motor
     */
    public double getRightRPM() {
        rightRPM = shooterFalcon2.getSelectedSensorVelocity() * 600.0 / (rightRatio * encoderEPR);
        return rightRPM;
    }

    /**
     * @return Updates the RPM value of the left shooter motor
     */
    public double getLeftRPM() {
        leftRPM = shooterFalcon1.getSelectedSensorVelocity() * 600.0 / (leftRatio * encoderEPR);
        return leftRPM;
    }

    /**
     * Checks if both of the rpms are greater than the rpm threshold
     */
    public boolean atSpeed() {
        return ((getRightRPM() >= (speed - rpmThreshold)) || (getLeftRPM() >= (speed - rpmThreshold)));
    }

    // TODO: Test this method's functionality
    /**
     * Checks if the motor voltages are greater than the acceptable voltage threshold
     */
    public boolean voltageSpike() {
        return ((shooterFalcon1.getBusVoltage() >= voltThreshold) || (shooterFalcon2.getBusVoltage() >= voltThreshold));
    }

    /**
     * Stops the shooter
     */
    public void stopShooter() {
        shooterFalcon1.stopMotor();
        shooterFalcon2.stopMotor();
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void setRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}