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


public class LiftModule extends SubsystemBase {

  private final WPI_TalonFX m_extendorRight;
  private final WPI_TalonFX m_extendorLeft;
/* 
  //intake rollers that sucks in the cone
  private final TalonSRX m_topRoller;
  private final TalonSRX m_bottomRoller;
  //intake rollers that sucks in the cone

  private final TalonSRX m_clawLeft;
  private final TalonSRX m_clawRight;
  private final TalonSRX m_clawRotation;

  private final DigitalInput m_liftExtended;
  private final DigitalInput m_liftContracted;
  */


  public LiftModule(int extendorRight, int extendorLeft, int topRoller,int bottomRoller,int clawLeft,
  int clawRight,int clawRotation, int liftExtended, int liftContracted ) {

 

    m_extendorRight = new WPI_TalonFX(extendorRight);
    m_extendorLeft = new WPI_TalonFX(extendorLeft);
/* 
    m_topRoller = new TalonSRX(topRoller);
    m_bottomRoller = new TalonSRX(bottomRoller);

    m_clawLeft = new TalonSRX(clawLeft);
    m_clawRight = new TalonSRX(clawRight);
    m_clawRotation = new TalonSRX(clawRotation);

    m_liftExtended = new DigitalInput(liftExtended);
    m_liftContracted = new DigitalInput(liftContracted);
    */

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    m_extendorRight.configAllSettings(configs);
    m_extendorLeft.configAllSettings(configs);

    m_extendorRight.setInverted(false);
    m_extendorLeft.setInverted(true);

    m_extendorLeft.follow(m_extendorRight);


    m_extendorRight.config_kP(0, LiftConstants.kLIFTP);
    m_extendorLeft.config_kP(0, LiftConstants.kLIFTP);

    m_extendorRight.config_kI(0, LiftConstants.kLIFTI);
    m_extendorLeft.config_kI(0, LiftConstants.kLIFTI);

    m_extendorRight.configAllowableClosedloopError(0, LiftConstants.kAllowableError);
    m_extendorLeft.configAllowableClosedloopError(0, LiftConstants.kAllowableError);



    m_extendorRight.setSelectedSensorPosition(0);
    m_extendorLeft.setSelectedSensorPosition(0);

    //m_extendorRight.

/* 
    m_topRoller.setInverted(false);
    m_bottomRoller.setInverted(false);

    m_clawLeft.setInverted(false);
    m_clawRight.setInverted(false);
    m_clawRotation.setInverted(false);
*/
    m_extendorRight.setNeutralMode(NeutralMode.Brake);
    m_extendorLeft.setNeutralMode(NeutralMode.Brake);
/* 
    m_topRoller.setNeutralMode(NeutralMode.Brake);
    m_bottomRoller.setNeutralMode(NeutralMode.Brake);

    m_clawLeft.setNeutralMode(NeutralMode.Brake);
    m_clawRight.setNeutralMode(NeutralMode.Brake);
    m_clawRotation.setNeutralMode(NeutralMode.Brake);
*/
  }

  public WPI_TalonFX getExtendorRight(){
    return m_extendorRight;
  }

  public WPI_TalonFX getExtendorLeft(){
    return m_extendorLeft;
  }
/* 
  public TalonSRX getTopRoller() {
    return m_topRoller;
  }

  public TalonSRX getBottomRoller() {
    return m_bottomRoller;
  }

  public TalonSRX getClawLeft() {
    return m_clawLeft;
  }

  public TalonSRX getClawRight() {
    return m_clawRight;
  }

  public TalonSRX getClawRotation() {
    return m_clawRotation;
  }

  public DigitalInput getLiftExtended() {
    return m_liftExtended;
  }

  public DigitalInput getLiftContracted() {
    return m_liftContracted;
  }
*/
  
}

/* 
class ClawModule extends SubsystemBase {

  private final TalonSRX m_clawLeft;
  private final TalonSRX m_clawRight;
  private final TalonSRX m_clawRotation;

  private final CANCoder m_wristEncoder;

  private final DigitalInput m_coneDetector;
  private final ColorSensorV3 m_boxDetector; 
  private final ColorMatch m_colorMatch;



  public ClawModule(int clawLeftID, int clawRightID,int clawRotationID, int wristID, int coneDetectorID ) {

    m_clawLeft = new TalonSRX(clawLeftID);
    m_clawRight = new TalonSRX(clawRightID);
    m_clawRotation = new TalonSRX(clawRotationID);

    m_wristEncoder = new CANCoder(wristID);

    m_clawLeft.setInverted(false);
    m_clawRight.setInverted(true);
    m_clawRotation.setInverted(false);

    m_clawLeft.setNeutralMode(NeutralMode.Brake);
    m_clawRight.setNeutralMode(NeutralMode.Brake);
    m_clawRotation.setNeutralMode(NeutralMode.Brake);

    m_coneDetector = new DigitalInput(coneDetectorID);
    addChild("ConeDectionBeam", m_coneDetector);

    m_boxDetector = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatch = new ColorMatch();
    m_colorMatch.addColorMatch(ColorConstants.kPurpleTarget);



  }

  public void intakeIn() {

    ColorMatchResult match = m_colorMatch.matchClosestColor(m_boxDetector.getColor());

   boolean m_coneDetectionBeam = m_coneDetector.get();


    if (match.color != ColorConstants.kPurpleTarget || m_coneDetectionBeam == false )
    {
      m_clawLeft.set(ControlMode.PercentOutput, 1);
      m_clawRight.set(ControlMode.PercentOutput, 1);
    }

    else
    {
      m_clawLeft.set(ControlMode.PercentOutput, 0);
      m_clawRight.set(ControlMode.PercentOutput, 0);
    }

  }

  public void intakeOut() {

  }

}
*/


