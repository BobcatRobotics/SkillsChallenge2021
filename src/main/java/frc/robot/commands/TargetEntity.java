package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class TargetEntity extends CommandBase {
	// Minimum power is reach at (1.351...) degrees from the center
	// degreesFromCenter = minimumPower / k
	private final double k = 0.037037; // Power constant (27 degrees = Power of 1)
	private final double minimumPower = 0.05; // Minimal power to send
	private final double threshold = 0.15; // The threshold in degrees where the turret won't move

	private Limelight limelight;
	private Turret turret;
	private Joystick gamepad;

	private double power; // Updated by the Limelight camera
	private double additionalPower; // Updated by the right Joystick on the gamepad
	private double turretPower; // Updated by the LimeLight camera; equal to power + additionalPower

	/**
	 * 1. Check if Limelight has a target<br>
	 * 2. Get the tx value <br>
	 * 3. Turn the value into a speed by inverting the X value and multiplying it by
	 * the power constant <br>
	 * 4. Tune the power to comply with the minimum power and threshold <br>
	 * 5. Get the additional power from the right stick's Y value <br>
	 * 6. If the total power is greater than 1, set it to 1 <br>
	 * 7. If the total power is less than -1, set it to -1 <br>
	 * 8. Pass the power to the motor <br>
	 * 9. Print the values to SmartDashboard
	 * 
	 * @param ll   Limelight subsystem
	 * @param trrt Turret subsystem
	 * @param gmpd XboxController instance
	 */
	public TargetEntity(Limelight ll, Turret trrt, Joystick gmpd) {
		turret = trrt;
		limelight = ll;
		gamepad = gmpd;
		power = 0;
		additionalPower = 0;
		turretPower = 0;
		addRequirements(limelight, turret);
	}

	/**
	 * Execute one iteration of the TargetEntity command (For multiple iterations,
	 * call multiple times) Only executes if it is enabled by the game pad
	 */
	@Override
	public void execute() {
		// If user is pressing right joystick in... target
		if (gamepad.getRawButton(Constants.Right_Joystick_Pressed)) {
			limelight.setLED(3); // Turn on the LED's if they haven't been turned on before
			limelight.setCAM(0); // Turn on vision mode if it wasn't turned on before

			if (limelight.hasTarget()) {
				updateTurretPower();
				turret.setSpinPower(turretPower);
			} else {
				power = 0;
				additionalPower = 0;
				turretPower = 0;
			}

			// Put values on the Smart Dashboard
			logValues();
		}
	}

	/**
	 * Retrive and preform calculations to calculate the turret power
	 */
	public void updateTurretPower() {
		double x = limelight.x(); // The number of degrees the target is off center horizontally
		double degreesToCenter = -x; // The number of degrees Limelight needs to shift by to be centered

		// Divide the degrees to center by 27
		// Ex. 27 degrees is a power of 1
		power = k * degreesToCenter;

		// Make sure the threshold and minimum power values are statisfied
		if (x > threshold && power > -minimumPower) {
			power = -minimumPower;
		} else if (x < -threshold && power < minimumPower) {
			power = minimumPower;
		} else if (x < threshold && x > -threshold) {
			power = 0.0;
		}

		// Increases the speed of the turret with the gamepad's right stick
		additionalPower = Math.abs(gamepad.getY(Hand.kRight)) * Math.signum(power);
		// Total power of the turret
		turretPower = power + additionalPower;
		// Makes sure the power isn't higher than 1 or lower than -1
		if (turretPower > 1) {
			turretPower = 1;
		} else if (turretPower < -1) {
			turretPower = -1;
		}
	}

	/**
	 * Log diagnostic values
	 */
	public void logValues() {
		SmartDashboard.putNumber("GamePad.POV", gamepad.getPOV());
		SmartDashboard.putNumber("Target.Power", power);
		SmartDashboard.putNumber("Target.AddPower", additionalPower);
		SmartDashboard.putNumber("Target.TurretPower", turretPower);
	}

	/**
	 * If the the down arrow on the POV is pressed, the command is done. Stops
	 * targeting and turns off the LEDs so people don't get blinded.
	 * 
	 * @param interrupted Whether the command was interrupted/canceled
	 */
	@Override
	public void end(boolean interrupted) {
		limelight.setLED(1); //TODO Fix all limeLight.setLED() calls to right things
	}
}
