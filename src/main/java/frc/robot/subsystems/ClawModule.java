// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ColorConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.sensors.CANCoder;


public class ClawModule extends SubsystemBase {

  private final TalonSRX m_clawLeft;
  private final TalonSRX m_clawRight;
  private final TalonSRX m_clawRotation;

  private final TalonSRX m_topRoller;
  private final TalonSRX m_bottomRoller;

  private final CANCoder m_wristEncoder;

  private final DigitalInput m_coneDetector;
  private final ColorSensorV3 m_boxDetector; 
  private final ColorMatch m_colorMatch;

  public ClawModule(int clawLeftID, int clawRightID,int clawRotationID, int wristID, int topRollerID, int bottomRollerID, int coneDetectorID ) {

    m_clawLeft = new TalonSRX(clawLeftID);
    m_clawRight = new TalonSRX(clawRightID);


    m_topRoller = new TalonSRX(topRollerID);
    m_bottomRoller = new TalonSRX(bottomRollerID);

    //Josh E REVIEW THESE LINES
    m_wristEncoder = new CANCoder(wristID);
    m_clawRotation = new TalonSRX(clawRotationID);

    m_wristEncoder.setPosition(0);

    m_clawRotation.config_kP(0,ClawConstants.kClawP);
    m_clawRotation.config_kI(0, ClawConstants.kClawI);
    m_clawRotation.config_kD(0, ClawConstants.kClawD);

    m_clawRotation.configAllowableClosedloopError(0, ClawConstants.allowableClawError);
    m_clawRotation.configRemoteFeedbackFilter(m_wristEncoder, 0);
    //Josh E REVIEW THESE LINES

    m_clawLeft.setInverted(true);
    m_clawRight.setInverted(false);
    m_clawRotation.setInverted(false);

    m_topRoller.setInverted(false);
    m_bottomRoller.setInverted(true);

    m_clawLeft.setNeutralMode(NeutralMode.Brake);
    m_clawRight.setNeutralMode(NeutralMode.Brake);
    m_clawRotation.setNeutralMode(NeutralMode.Brake);
    m_bottomRoller.setNeutralMode(NeutralMode.Brake);
    m_topRoller.setNeutralMode(NeutralMode.Brake);

    m_coneDetector = new DigitalInput(coneDetectorID);
    addChild("ConeDectionBeam", m_coneDetector);

    m_boxDetector = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatch = new ColorMatch();
    m_colorMatch.addColorMatch(ColorConstants.kPurpleTarget);

  }

  public TalonSRX getclawLeft() {
    return m_clawLeft;
  }

  public TalonSRX getclawRight() {
    return m_clawRight;
  }

  public TalonSRX getclawRotation() {
    return m_clawRotation;
  }

  public TalonSRX gettopRoller() {
    return m_topRoller;
  }

  public TalonSRX getbottomRoller() {
    return m_bottomRoller;
  }

  public CANCoder getwristEncoder() {
    return m_wristEncoder;
  }

  public DigitalInput getconeDetector() {
    return m_coneDetector;
  }

  public ColorSensorV3 getboxDetector() {
    return m_boxDetector;
  }

  public ColorMatch getcolorMatch() {
    return m_colorMatch;
  }


  
}



