package frc.robot.subsystems;

import static frc.robot.Constants.ColorWheelConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase {
    private final WPI_TalonSRX colorWheelMotor;

    private double spinPower;

    public ColorWheel() {
        colorWheelMotor = new WPI_TalonSRX(colorWheelMotorPort);
        // When the motor is in neutral mode the motor will keep moving easily (coast)
        colorWheelMotor.setNeutralMode(NeutralMode.Brake);

    }

    public void spin() {
        colorWheelMotor.set(.5);
    }

    public void stop() {
        colorWheelMotor.stopMotor();
    }
}