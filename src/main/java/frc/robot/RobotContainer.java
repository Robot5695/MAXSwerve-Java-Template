// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

import com.ctre.phoenix6.configs.PWM1Configs;

import frc.robot.commands.AlignToReef;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.targetFollow;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Climber;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
  private final Elevator m_Elevator = new Elevator();
  private final Tilter m_Tilter = new Tilter();
  private final Climber m_Climber= new Climber();



  private double targetDelta;
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController2 = new CommandXboxController(OIConstants.kDriverControllerPort+1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    m_chooser.setDefaultOption("moveOnly", moveOnly());
    m_chooser.addOption ("Center", tagfollower(2));
    m_chooser.addOption ("Right", tagfollower(2));
    m_chooser.addOption ("Left", tagfollower(2));
    SmartDashboard.putData(m_chooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false),
            m_robotDrive)); 

            //roller defaults to stopped, reverse is direction of coral feed
    m_rollerSubsystem.setDefaultCommand(
        new RollerCommand(
          () -> 0.0,
            () -> 0.0,
           
            m_rollerSubsystem));

    
    //elevator default logic
    m_Elevator.setDefaultCommand(
      new RunCommand(()->m_Elevator.setTarget(m_Elevator.getTarget()+(m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis())/2), m_Elevator)
    );


    //tilter default logic
    m_Tilter.setDefaultCommand(
      new RunCommand(()->m_Tilter.setTilterSpeed(0),m_Tilter)
    );
    //climber default logic
    m_Climber.setDefaultCommand(
      new RunCommand(()->m_Climber.setClimberSpeed(0),m_Climber)
    );
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController
    .x()
    .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));

    /* 
    //commands to align to the reef
    m_driverController.rightTrigger().onTrue(new AlignToReef(true, m_robotDrive).withTimeout(7));
    m_driverController.leftTrigger().onTrue(new AlignToReef(false, m_robotDrive).withTimeout(7));
    */
        
    m_driverController2
    .rightBumper()
    .whileTrue(new RollerCommand(
      () -> 0.0,
        () -> 1.0,
        m_rollerSubsystem));

    m_driverController2
    .leftBumper()
    .whileTrue(new RollerCommand(
      () -> 0.0,
        () -> 0.5,
        m_rollerSubsystem));
   
        //0 is bottom, 57 is absolute top
m_driverController2.a().whileTrue(new RunCommand(()->m_Elevator.setTarget(0), m_Elevator));
m_driverController2.b().whileTrue(new RunCommand(()->m_Elevator.setTarget(15),m_Elevator));
m_driverController2.x().whileTrue(new RunCommand(()->m_Elevator.setTarget(30), m_Elevator));
m_driverController2.y().whileTrue(new RunCommand(()->m_Elevator.setTarget(55), m_Elevator));


m_driverController2.povLeft().whileTrue(new RunCommand(()->m_Tilter.setTilterSpeed(0.5),m_Tilter));
m_driverController2.povRight().whileTrue(new RunCommand(()->m_Tilter.setTilterSpeed(-0.5),m_Tilter));

        m_driverController2.leftStick().whileTrue(new RunCommand(()->m_Climber.setClimberSpeed(-1), m_Climber));
        m_driverController2.rightStick().whileTrue(new RunCommand(()->m_Climber.setClimberSpeed(1), m_Climber));
        /* bumper based elevator control
        m_driverController
        .rightBumper()
        .whileTrue(new RunCommand(() -> m_Elevator.setTarget(m_Elevator.getTarget()+1), m_Elevator));

        m_driverController
        .leftBumper()
        .whileTrue(new RunCommand(() -> m_Elevator.setTarget(m_Elevator.getTarget()-1), m_Elevator));
        */
  }

  public Command getAutonomousCommand(){

    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command moveOnly() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }


  public Command tagfollower(int pipeline){
LimelightHelpers.setPipelineIndex("limelight-front", pipeline);
    return new targetFollow(m_robotDrive,m_rollerSubsystem, m_Elevator);
  }
}
