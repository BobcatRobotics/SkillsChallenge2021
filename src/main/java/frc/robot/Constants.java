/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  /* CONSTANTS INFO
   * From the official wpilib docs:
   * "It is recommended that users separate these constants 
   * into individual inner classes corresponding to subsystems or robot modes,
   * to keep variable names shorter."
   * "In Java, all constants should be declared public static final
   * so that they are globally accessible and cannot be changed."
   * "In Java, it is recommended that the constants be used from other classes
   * by statically importing the necessary inner class."
   */

  //Button definitions for gamepad
  public static final int A_Button = 2;
  public static final int B_Button = 3;
  public static final int X_Button = 1;
  public static final int Y_Button = 4;

  public static final int Right_Bumper_Button = 6;
  public static final int Right_Trigger_Button = 8;
  public static final int Left_Bumper_Button = 5;
  public static final int Left_Trigger_Button = 7;

  public static final int Left_Joystick_Pressed = 11;
  public static final int Right_Joystick_Pressed = 12;


  // public static final int Left_Joystick_X_Axis = 1;
  // public static final int Left_Joystick_Y_Axis = 0;

  // public static final int Right_Joystick_X_Axis= 13;
  // public static final int Right_Joystick_Y_Axis= 14;

  public static final int D_Pad_Up = 0;
  public static final int D_Pad_Down = 180;
  public static final int D_Pad_Left = 270;
  public static final int D_Pad_Right = 90;


  public static final String SHOOT_SCOOT = "Shoot & Scoot";
  public static final String SCOOT_N_SHOOT = "Scoot & Shoot";
  public static final String TRENCH_RUN = "Trench Run";
  public static final String PUSH = "PUSH";
  public static final String DONT_PUSH = "DONT_PUSH";

  public static final int RS_Shift_Switch = 3;
  

  /**
   * Constants for the climber subsystem
   */
   
  public static final class ClimberConstants
  {
    public static final int winchMotorPort = 8;
    public static final int climberSolenoidPort = 2;
  }

   /**
    * Constants for the drive train
    */
  public static final class DrivetrainConstants {

    public static final double INVERT_MOTOR = -1.0;
    // Drive Train motors
    public static final int LTMotorPort = 0;
    public static final int LMMotorPort = 1;
    public static final int LLMotorPort = 2;
    public static final int RTMotorPort = 3;
    public static final int RMMotorPort = 4; 
    public static final int RLMotorPort = 5;

    // Need to edit all the numbers under this for 2020
    public static final int[] leftEncoderPorts = new int[] { 2, 3 };
    public static final int[] rightEncoderPorts = new int[] { 0, 1 };
    public static final boolean leftEncoderReversed = false;
    public static final boolean rightEncoderReversed = false;

    public static final int encoderCPR = 128;
    public static final double wheelDiameterMeters = 0.1;
    public static final double encoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (wheelDiameterMeters * Math.PI) / (double) encoderCPR;

    public static final boolean GYRO_REVERSED = true;
  }

  /**
   * Constants for the feeder system
   */
  public static final class FeederConstants {
    public static final double INVERT_MOTOR = -1.0;

    // Ball Present Sensor Config
    public static final int feederBallPresentId = 0;
    public static final double feederBallPresentThreshold = 125.0;

    // Power cell feeder motors
    public static final int feederMotor1Port = 11;

    // Makes sure the speed does not increase over this number
    public static final double speedLimiter = -0.7;

    public static final double speedLimiterSlow = -0.6;

    public static final int feederBallPresentSensor = 17;
  }

  /**
   * Constants for the intake system
   */
  public static final class IntakeConstants {
    //Ball Present Sensor Config
    public static final int intakeBallPresentSensor = 16;
    public static final double intakeBallPresentThreshold = 125.0;
    // Power cell intake motor
    public static final int intakeBarMotorPort = 9; 
    public static final int intakeFunnelMotorPort = 6;
    public static final int intakeLowerTowerFalconPort = 12;
    public static final int intakeSolenoidPort = 3;
  }

  /**
   * Constants for the input device ports
   */
  public static final class RobotContainerConstants {
    // Sticks
    public static final int rightStickPort = 1;
    public static final int leftStickPort = 0;
    public static final int gamepadPort = 2;

    // Gamepad POV values in degrees
    // public static final int povNone = -1;
    public static final int povUp = 0;
    // public static final int povUpperRight = 45;
    public static final int povRight = 90;
    // public static final int povLowerRight = 135;
    public static final int povDown = 180;
    // public static final int povLowerLeft = 225;
    public static final int povLeft = 270;
    // public static final int povUpperLeft = 315;
  }

  /**
   * Constants for path finding and trajectories
   */
  public static final class RouteFinderConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these values for your robot.
    public static final double ksVolts = 1.43;
    public static final double kvVoltSecondsPerMeter = 5.15;
    public static final double kaVoltSecondsSquaredPerMeter = 0.384;
    public static final double kTrackwidthMeters = 0.77;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = -12.5;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // On-the-go route finding waypoint variables
    public static final int pointx = 0;
    public static final int pointy = 0;
    public static final int rotation = 0;
  }

  /**
   * Constants for the shooter subsystem
   */
  public static final class ShooterConstants {
    // Shooter spinner motors
    public static final int shooterFalcon1Port = 13;
    public static final int shooterFalcon2Port = 14;
    public static final int shooterAngleSolenoid = 1;
  }

  /**
   * Constants for the turret subsystem
   */
  public static final class TurretConstants {
    // Shooter spinner and turret moving motors
    public static final int turretMotorPort = 10;
    public static final double turretMotorHardStopCurrentThreshold = 10.0;
    public static final int turretSolenoidPort = 0;

  }

  /**
   * Constants for the color wheel subsystem
   */
  public static final class ColorWheelConstants {
    public static final int colorWheelMotorPort = 8;

  }

  public static final class StopAtCollisionConstants {
      public static boolean collision = true;
      public static final double kCollisionThreshold_DeltaG = 1.2; 
  }
}
