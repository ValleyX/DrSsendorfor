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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SequentialCommandGroupPlaybackBalance;
import frc.robot.commands.SequentialCommandGroupPlaybackRecord;
import frc.robot.commands.TelopCommand;
import frc.robot.commands.TelopCommand.DriveType;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LiftModule;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.extendPosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SequentialCommandGroupPlaybackRecord m_SeqCmdPlaybackLeft;
  private final SequentialCommandGroupPlaybackRecord m_SeqCmdPlaybackRight;
  private final SequentialCommandGroupPlaybackBalance m_SeqCmdPlaybackBalance;
 // private final SequentialCommandGroupPlaybackRecord m_SeqCmdRecord;
 // private final SequentialCommandGroupPlaybackRecord m_SeqCmdRecordBalance;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClawSubsystem m_ClawSubsystem = new ClawSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem(m_ClawSubsystem);
  

  // The driver's controller

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_SeqCmdPlaybackLeft = new SequentialCommandGroupPlaybackRecord(DriveType.Playback, 15, "LeftRecord", m_robotDrive, m_liftSubsystem, m_ClawSubsystem);
    m_SeqCmdPlaybackRight = new SequentialCommandGroupPlaybackRecord(DriveType.Playback, 15, "RightRecord", m_robotDrive, m_liftSubsystem, m_ClawSubsystem);
    m_SeqCmdPlaybackBalance = new SequentialCommandGroupPlaybackBalance(DriveType.Playback, 13, "MiddleRecord", m_robotDrive, m_liftSubsystem, m_ClawSubsystem);
  
    //rename this between runs for specific type
  //  m_SeqCmdRecord = new SequentialCommandGroupPlaybackRecord(DriveType.Record, 15, "LeftRecord", m_robotDrive, m_liftSubsystem);
  //  m_SeqCmdRecordBalance = new SequentialCommandGroupPlaybackRecord(DriveType.Record, 13, "MiddleRecord", m_robotDrive, m_liftSubsystem);
 

    //LiftSubsystem.extendPosition requestedPostion = LiftSubsystem.extendPosition.low;


    // Configure the button bindings
   // configureButtonBindings();
/* 
    m_liftSubsystem.setDefaultCommand(

        new RunCommand(
            () -> m_liftSubsystem.liftDrive(
                m_manipulatorController.getLeftY(), 
                0,//m_manipulatorController.getPOV(LiftConstants.kPOIRotationDOWN), 
                0,//m_manipulatorController.getPOV(LiftConstants.kPOIRotationUP), 
                m_manipulatorController.getYButton(),
                m_manipulatorController.getBButton(),
                m_manipulatorController.getAButton()), 
            m_liftSubsystem)

    );
    */
/* 
    //Works
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
    */
    //experimental
    //Only runs this when in telop mode
    m_robotDrive.setDefaultCommand(new TelopCommand(DriveType.Telop, 0, "",  m_robotDrive, m_liftSubsystem, m_ClawSubsystem));
   // m_robotDrive.setDefaultCommand(new TelopCommand(DriveType.Record, 15, "LeftRecord",  m_robotDrive, m_liftSubsystem));
        
    
    //chooser run in auto mode
    m_chooser.setDefaultOption("Left", m_SeqCmdPlaybackLeft);
    m_chooser.addOption("Right",       m_SeqCmdPlaybackRight);
    m_chooser.addOption("Banlace",       m_SeqCmdPlaybackBalance);
    //m_chooser.addOption("Record Full 15 secs",  m_SeqCmdRecord);
    //m_chooser.addOption("Record 13 secs",  m_SeqCmdRecordBalance);

    SmartDashboard.putData("Auto Mode", m_chooser);
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
  /* 
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            
  }
  */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public DriveSubsystem getDriveSystem(){
    return m_robotDrive;
  }

  public LiftSubsystem getLiftSystem(){
    return m_liftSubsystem;
  }
}
