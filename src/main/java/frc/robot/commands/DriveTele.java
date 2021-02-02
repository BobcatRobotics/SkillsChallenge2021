package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTele extends CommandBase {

    // If the speed of a motor is less than this threshold, we'll just set it to zero
    private final double driveThreshold = 0.07;

    // Represents the position of the right and left joysticks
    // These determine the speed of the right and left motors
    private double right = 0.0;
    private double left = 0.0;

    private final Drivetrain drivetrain;
    private final Joystick rightStick;
    private final Joystick leftStick;

    /**
     * 1. This command drives the robot. <br>
     * 2. It receives inputs from the left and right joysticks and sets the speed of
     * the motors. <br>
     * 3. Pushing the left stick forward makes the left wheels go forward. <br>
     * 4. Pushing the right stick forward makes the right wheels go forward. <br>
     * 5. First, it reads the values from the left and right joysticks. <br>
     * 6. If they are within +/- 0.08 of 0, then it just sets the value to 0. <br>
     * 7. Then, it sets the power of the right motors, then the left motors. <br>
     * 8. It takes the value from the sticks and makes sure they are within the
     * interval [-1, 1]. <br>
     * 9. If it is greater than 1 it sets it to 1, if it is less than -1 it sets it to -1. <br>
     * 10. Then, it scales the power value by taking the square root of the value. <br>
     * 11. If the power is negative it returns the negative square root of the
     * negative value. <br>
     * 12. This command is perpetually active, but the robot cannot drive during autonomous mode.
     */
    public DriveTele(Drivetrain drivetrain, Joystick rightStick, Joystick leftStick) {
        this.drivetrain = drivetrain;
        this.rightStick = rightStick;
        this.leftStick = leftStick;
        addRequirements(drivetrain);
    }

    /**
     * Sets the motor speed values to the values from the right and left joysticks
     * It can only drive if it is teleoperated mode.
     */
    @Override
    public void execute() {
        if (RobotState.isOperatorControl()) {
            // Gets the values of the right and left joystick positions
            right = rightStick.getRawAxis(Joystick.AxisType.kY.value);
            left = leftStick.getRawAxis(Joystick.AxisType.kY.value);

            // If the value of the joysticks are too low just set it to zero
            if (Math.abs(right) <= driveThreshold) {
                right = 0.0;
            }

            if (Math.abs(left) <= driveThreshold) {
                left = 0.0;
            }

            drivetrain.drive(right, left);
        } else {
            drivetrain.stop();
        }
    }

    // Runs when command is interrupted
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}