/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.RobotContainerConstants.gamepadPort;
import static frc.robot.Constants.RobotContainerConstants.leftStickPort;
import static frc.robot.Constants.RobotContainerConstants.rightStickPort;
import static frc.robot.Constants.RouteFinderConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.RouteFinderConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.RouteFinderConstants.kTrackwidthMeters;
import static frc.robot.Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.RouteFinderConstants.ksVolts;
import static frc.robot.Constants.RouteFinderConstants.kvVoltSecondsPerMeter;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret; 
import frc.robot.subsystems.NavxGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;

public class RobotContainer {
  // Subsystems should be private here and have to be passed to commands because it is better coding practice.

  // Joysticks
  public static final Joystick rightStick = new Joystick(rightStickPort);
  public static final Joystick leftStick = new Joystick(leftStickPort);
  public static final Joystick gamepad = new Joystick(gamepadPort);
  
  // Drivetrain
  public static final Drivetrain drivetrain = new Drivetrain();

  // Shooter
  // public static final Shooter shooter = new Shooter();

  // // Feeder
  // public static final Feeder feeder = new Feeder();

  // // Intake
  // public static final Intake intake = new Intake();

  // // Limelight
  // public static final Limelight limelight = new Limelight();

  // // Turret
  // public static final Turret turret = new Turret(limelight);

  // //Climber
  // public static final Climber climber = new Climber();

  //Gyro
  public static final NavxGyro navx = new NavxGyro(SPI.Port.kMXP);

  //Color Wheel
  // public static final ColorWheel colorwheel = new ColorWheel();
  public static final ColorWheel colorwheel = null;

  //Trajectory
  public static Trajectory trajectory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     /**
     * COMMAND INFO From the official wpilib docs: "While users are able to create
     * commands by explicitly writing command classes (either by subclassing
     * CommandBase or implementing Command), for many commands (such as those that
     * simply call a single subsystem method) this involves a lot of wasteful
     * boilerplate code. To help alleviate this, many of the prewritten commands
     * included in the command-based library may be inlined - that is, the command
     * body can be defined in a single line of code at command construction."
     * 
     * TLDR: You shouldn't create a whole new file for a command that only calls one
     * method.
     * 
     * (I didn't know what these did so here is an explanation if anyone is
     * confused) Also method references and lambda expressions let you pass
     * subroutines as parameters Method reference: subsystem::method Lambda
     * expression: () -> subsystem.method() They essentially do the same thing
     */
    // Attaches a commmand to each button
    // Starts the shooter motors when the Y button is pressed
    // new JoystickButton(gamepad, Constants.Left_Bumper_Button).whenPressed(new RunShooter(shooter, feeder, limelight, gamepad));
    // Takes in balls from the ground when the right trigger is held
    // new JoystickButton(gamepad, Constants.Right_Bumper_Button).whenHeld(new IntakeIn(intake,gamepad));
    // new JoystickButton(gamepad, Constants.Right_Bumper_Button).whenReleased(new IntakeStop(intake,gamepad));
    // // Pushes out balls onto the ground when the right bumper is held
    // new JoystickButton(gamepad, Constants.Right_Trigger_Button).whenHeld(new IntakeOut(intake,gamepad));
    // new JoystickButton(gamepad, Constants.Right_Trigger_Button).whenReleased(new IntakeStop(intake,gamepad));
    //Run the feeder system
    // new JoystickButton(gamepad, Constants.Left_Trigger_Button).whenHeld(new FeederRun(feeder, intake, shooter, gamepad));
    // //Lift Intake pneau
    // new JoystickButton(gamepad, Constants.X_Button).whenPressed(new IntakeUp(intake));
    // //lower intake pneau
    // new JoystickButton(gamepad, Constants.Y_Button).whenPressed(new IntakeDown(intake));

    // new JoystickButton(gamepad, Constants.Left_Joystick_Pressed).whenHeld(new WithdrawClimber(climber, gamepad));

    // new JoystickButton(gamepad,Constants.Left_Joystick_Pressed).whenReleased(new StopClimber(climber));
    // // Starts targeting when the up arrow on the D-pad is pressed
    // new POVButton(gamepad, povUp).whenPressed(new DeployClimber(climber, gamepad));
    // // Ends targeting when the down arrow on the D-pad is pressed
    // new POVButton(gamepad, povDown).whenHeld(new ClearAllBalls(intake, feeder, shooter, gamepad));
    // new POVButton(gamepad, povDown).whenReleased(new StopBelts(intake, feeder, shooter, gamepad));
    //raise shooter angle
    // new POVButton(gamepad, povRight).whenPressed(new PancakeUp(shooter, gamepad));
    //lower shooter angle
    // new POVButton(gamepad, povLeft).whenPressed(new PancakeDown(shooter, gamepad));
    // Heads to a position when the left bumper is pressed
    // new JoystickButton(gamepad, Button.kBumperLeft.value)
    //     .whenPressed(RouteFinder.getPathCommand(RouteFinder.trajectorygen(pointx, pointy, rotation)));
    
    // Driving
    // new PerpetualCommand(new DriveTele(drivetrain, rightStick, leftStick)).schedule();
    // Turret
   // new PerpetualCommand(new MoveTurret(turret, gamepad)).schedule(); 
  }

  public static TrajectoryConfig getConfig() {
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, 10);

    // Create config for trajectory
    return new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        // Doesn't reverse the trajectory
        .setReversed(false);
  }

  public Command getBarrelAutonomousCommand() {
    String trajectoryJSON = "paths/testing.wpilib.json";
    Trajectory trajectory = new Trajectory();	      
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
      DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts,
                                      Constants.RouteFinderConstants.kvVoltSecondsPerMeter,
                                       Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);
    Transform2d trans = drivetrain.getPose().minus(trajectory.getInitialPose());
      //TrajectoryConfig config = new TrajectoryConfig(3, 3).setKinematics(kDriveKinematics).addContraint(autoVoltageConstraint);
    trajectory.transformBy(trans);
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, // We input our desired trajectory here
    drivetrain::getPose, new RamseteController(Constants.RouteFinderConstants.kRamseteB, Constants.RouteFinderConstants.kRamseteZeta),
    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
    drivetrain::getWheelSpeeds, new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0), new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::tankDriveVolts, drivetrain);
    return ramseteCommand.beforeStarting(drivetrain::resetOdometry).andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  public Command getBounceAutonomousCommand(){

    String trajectoryJSON = "paths/bounce.wpilib.json";	      
    	
    Trajectory trajectory = new Trajectory();      
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
      DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts,
                                      Constants.RouteFinderConstants.kvVoltSecondsPerMeter,
                                       Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);

    RamseteCommand bounceCommand = new RamseteCommand(trajectory, // We input our desired trajectory here
    drivetrain::getPose, new RamseteController(Constants.RouteFinderConstants.kRamseteB, Constants.RouteFinderConstants.kRamseteZeta),
    new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts, Constants.RouteFinderConstants.kvVoltSecondsPerMeter, Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter), kDriveKinematics,
    drivetrain::getWheelSpeeds, new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0), new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::tankDriveVolts, drivetrain); 
    return bounceCommand.beforeStarting(drivetrain::resetOdometry).andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  public Command getSlalomAutonomousCommand(){

    String trajectoryJSON = "paths/slalom.wpilib.json";	
    Trajectory trajector = new Trajectory();      
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts,
                                    Constants.RouteFinderConstants.kvVoltSecondsPerMeter,
                                     Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
          kDriveKinematics,
          10);

    RamseteCommand slalomCommand = new RamseteCommand(trajectory, // We input our desired trajectory here
    drivetrain::getPose, new RamseteController(Constants.RouteFinderConstants.kRamseteB, Constants.RouteFinderConstants.kRamseteZeta),
    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
    drivetrain::getWheelSpeeds, new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0), new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::tankDriveVolts, drivetrain); 

    return slalomCommand.beforeStarting(drivetrain::resetOdometry).andThen(() -> drivetrain.drive(0, 0));

  }

public Command getAutonomousCommand(String path, int i) {
  var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  // Trajectory
  String trajectoryJSON = path; // Set to "" if it doesn't work
  // Create a voltage constraint to ensure we don't accelerate too fast
  Trajectory trajectory = new Trajectory();
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts,
                                  Constants.RouteFinderConstants.kvVoltSecondsPerMeter,
                                   Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        10);
  RamseteCommand ramseteCommand = new RamseteCommand(trajectory, // We input our desired trajectory here
    drivetrain::getPose, new RamseteController(Constants.RouteFinderConstants.kRamseteB, Constants.RouteFinderConstants.kRamseteZeta),
    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
    drivetrain::getWheelSpeeds, new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0), new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    drivetrain::tankDriveVolts, drivetrain);
  return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  // if ( i % 2 == 1) {
  //   ParallelCommandGroup endRun = new ParallelCommandGroup(
  //     FeederStopAuto(feeder,intake,shooter),
  //     drivetrain.tankDriveVolts(0, 0)
  //   );
  //   return ramseteCommand.beforeStarting(() -> drivetrain.tankDriveVolts(0, 0)).alongWith(FeederRunAuto(feeder, intake, shooter).feed()).andThen(() -> endRun);
  // } else{
  //   return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  // }
}

public Command getStartAutonomousCommand(String path) {
  var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  // Trajectory
  String trajectoryJSON = path; // Set to "" if it doesn't work
  // Create a voltage constraint to ensure we don't accelerate too fast
  Trajectory trajectory = new Trajectory();
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
  }
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts,
                                  Constants.RouteFinderConstants.kvVoltSecondsPerMeter,
                                   Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        10);
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, // We input our desired trajectory here
      drivetrain::getPose, new RamseteController(Constants.RouteFinderConstants.kRamseteB, Constants.RouteFinderConstants.kRamseteZeta),
      new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
      drivetrain::getWheelSpeeds, new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0), new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      drivetrain::tankDriveVolts, drivetrain);


    return ramseteCommand.beforeStarting(drivetrain::resetOdometry).andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}