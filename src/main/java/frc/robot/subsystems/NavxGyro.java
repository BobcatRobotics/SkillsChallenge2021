package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.lib.RioLogger;

public class NavxGyro extends AHRS {
	private PIDController turnController;
	private double toleranceDegrees = 2.0;
	private double rotateToAngleRate;

	/**
	 * This detects the rotation on 3 axes; roll, pitch, and yaw
	 * using the attitude and heading reference system (AHRS).
	 * Yaw: Rotation left and right about an axis running up and down;
	 * Pitch: Rotation up or down about an axis running from side to side;
	 * Roll: Rotation about an axis running from the front to the back.
	 */
	public NavxGyro(Port spiPortID) throws RuntimeException {
		super(spiPortID);
	}

	public void setToleranceDegrees(double toleranceDegrees) {
		this.toleranceDegrees = toleranceDegrees;
	}

	public double getRotateToAngleRate() {
		return rotateToAngleRate;
	}

	public void calibrate() {
		try {
			int maxCalibrationPasses = 20;
			for (int iCalibrationPasses = 0; iCalibrationPasses < maxCalibrationPasses; iCalibrationPasses++) {
				if (!isCalibrating()) {
					break;
				}
				RioLogger.log("robotInit() gyro is calibrating, pass " + iCalibrationPasses);
				try {
					Thread.sleep(100); // Sleep 1/10 of second
				} catch (InterruptedException e) {
					RioLogger.errorLog("navX-MXP initialization thread exception " + e);
				}
			}

			RioLogger.log("robotInit() gyro is calibrating " + isCalibrating());
			if (!isCalibrating()) {
				zeroYaw();
			}
			RioLogger.log("robotInit() currentYaw " + getYaw());
		} catch (RuntimeException ex) {
			RioLogger.errorLog("navX-MXP initialization error " + ex);
		}
	}
}