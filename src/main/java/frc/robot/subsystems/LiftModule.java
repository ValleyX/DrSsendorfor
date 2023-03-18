// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
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
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.sensors.CANCoder;

/************************************************************************************** */
/*class for lift system and others related to it     */
//
/************************************************************************************** */
public class LiftModule extends SubsystemBase {

  private final WPI_TalonFX m_extendorRight; //rights motor that moves the lift up/down
  private final WPI_TalonFX m_extendorLeft; //left motor that moves the lift up/down

  private DigitalInput m_liftLimitSwitchLeft; //stops the lift from going back too far
  private DigitalInput m_liftLimitSwitchRight; //stops the lift from going back too far (not being used currently)

  private Solenoid m_coneDeporter; //fires the pneumatic cone tipper (top bar)
  private PneumaticHub m_pneumaticHub; //control hub for the pneumatic system
  private TalonFXConfiguration m_configs ;
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

  /************************************************************************************** */
  /*constructor for the lift moduble which runs the lift up and down     */
  //extendorRight = the CAN ID for the right lift motor
  //extendorLeft = the CAN ID for the left lift motor
  //liftLimitSwitchLeft = digital input which detects whether you've gone back to far
  //pneumaticCANID = the CAN ID for the pneumatic system
  //pneumaticPortID = the port for the pneumatic system for the solenoid
  /************************************************************************************** */
  public LiftModule(int extendorRight, int extendorLeft, int liftLimitSwitchLeft, int liftLimitSwitchRight, int pneumaticCANID, int pneumaticPortID) 
  {
    m_extendorRight = new WPI_TalonFX(extendorRight); //contructs the object for the right lift motor
    m_extendorLeft = new WPI_TalonFX(extendorLeft); //constructs the object for the left lift motor

    m_liftLimitSwitchLeft = new DigitalInput(liftLimitSwitchLeft); //contructs the object for the limit switch
    m_liftLimitSwitchRight = new DigitalInput(liftLimitSwitchRight); //contructs the object for the limit switch

   m_configs = new TalonFXConfiguration();
   m_extendorRight.getAllConfigs(m_configs);
   m_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
   
   m_configs.slot0.closedLoopPeakOutput = .8;  //reduce max speed to 80%
   //m_extendorRight.getClosedLoopError();
  // m_extendorRight.setIntegralAccumulator(0);
  // m_configs.slot0.maxIntegralAccumulator

    m_extendorRight.configAllSettings(m_configs);
    m_extendorLeft.configAllSettings(m_configs);
    
   

    m_extendorRight.setInverted(false);
    m_extendorLeft.setInverted(true);

    m_extendorLeft.follow(m_extendorRight);

    //Set Cmd Ramp Rate - JAE 3/14/23
    //m_extendorRight.configClosedloopRamp(1);
    //m_extendorLeft.configClosedloopRamp(1);
    
    //proportional gain for lift motors for position control
    m_extendorRight.config_kP(0, LiftConstants.kLIFTP);
    m_extendorLeft.config_kP(0, LiftConstants.kLIFTP);

    //integral gain for lift motors for position control
    m_extendorRight.config_kI(0, LiftConstants.kLIFTI);
    m_extendorLeft.config_kI(0, LiftConstants.kLIFTI);

    //deadband for the postion
    m_extendorRight.configAllowableClosedloopError(0, LiftConstants.kAllowableError);
    m_extendorLeft.configAllowableClosedloopError(0, LiftConstants.kAllowableError);

    //resets the internal position sensors
    m_extendorRight.setSelectedSensorPosition(0);
    m_extendorLeft.setSelectedSensorPosition(0);

    //when you aren't applying power, brake
    m_extendorRight.setNeutralMode(NeutralMode.Brake);
    m_extendorLeft.setNeutralMode(NeutralMode.Brake);

    //creates the object for the penumatic hub
   m_pneumaticHub = new PneumaticHub(pneumaticCANID);
 // m_pneumaticHub.enableCompressorAnalog(0,120);
 
  //creates the solenoid object and sets it off
  m_coneDeporter = m_pneumaticHub.makeSolenoid(pneumaticPortID);
  m_coneDeporter.set(false);


  }

  //returns the reference for the right lift motor
  public WPI_TalonFX getExtendorRight(){
    return m_extendorRight;
  }

  //returns the reference for the left lift motor
  public WPI_TalonFX getExtendorLeft(){
    return m_extendorLeft;
  }

  //returns the the reference for the limit switch
  public DigitalInput getLiftLimitSwitchLeft(){
    return m_liftLimitSwitchLeft;
  }

  //returns the the reference for the limit switch
  public DigitalInput getLiftLimitSwitchRight(){
    return m_liftLimitSwitchRight;
  }

  //returns the reference for the solenoid
  public Solenoid getconeDeporter() {
    return m_coneDeporter;
  }

  //returns the reference for the pneumatic hub
  public PneumaticHub getPneumaticHub(){
    return m_pneumaticHub;
  }

  public TalonFXConfiguration getconfigs(){
    return m_configs;
  }

  //resets the integral on the lift so the arm doesn't do the funky overshooting thing
  public void resetIntegral() {
    m_extendorRight.setIntegralAccumulator(0);
  }

  public double getPIDError() { //retruns the current error so that it can be used in logic
    return m_extendorRight.getClosedLoopError();
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

