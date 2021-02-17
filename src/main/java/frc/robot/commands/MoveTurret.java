package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {
    
    // If the speed of the motor is less than this threshold, we'll just set it to zero
    public static final double percentToTurnMotorWhenCommandedByButton = .05;

    private final Turret turret;
    private final XboxController gamepad;

    /**
     * 1. Comply with limit switchs <br>
     * 3. Takes power value from the left joystick on the game pad <br>
     * 4. Comply with minimum power threshold <br>
     * 5. Set the turret's speed
     * 
     * @param trrt Turret subsystem
     * @param gmpd XboxController instance
     */
    public MoveTurret(Turret trrt, XboxController gmpd) {
        this.turret = trrt;
        this.gamepad = gmpd;
        addRequirements(turret);
    }
    
    @Override
    public void execute() {
        double rightJoyStickXAxis = gamepad.getRawAxis(Joystick.AxisType.kZ.value);
        boolean rightJoyStickDown = gamepad.getRawButton(Constants.Right_Joystick_Pressed);

        //If right is NOT pressed down AND joystick x axis is moved (disregarding nominal movement)
        //Is .02 a good threshold?
        if (!rightJoyStickDown && (rightJoyStickXAxis < -.02 || rightJoyStickXAxis > .02)) {
            //Is this too lower, high?
            if ((turret.getPosition() > 10650) && (rightJoyStickXAxis > 0.0)) {
                turret.setSpinPower(0.0);
            } else if ((turret.getPosition() < -10650) && (rightJoyStickXAxis < 0.0)) {
                turret.setSpinPower(0.0);
            } else {
                turret.setSpinPower(rightJoyStickXAxis * .4);
            }
        }
        else if(!rightJoyStickDown && !(rightJoyStickXAxis < -.02 || rightJoyStickXAxis > .02)){
            turret.setSpinPower(0.0);
        }        
    }

    /**
     * Stops the turret when the command ends
     */
    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
    }
}