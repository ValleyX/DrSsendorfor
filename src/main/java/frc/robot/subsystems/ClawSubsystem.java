// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.extendPosition;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

  //make members
  LiftConstants m_LiftConstants = new LiftConstants();
  LiftSubsystem.extendPosition m_lifeExtendState;
 // public double liftHeight = m_LiftConstants.kExtendorPositionlow;

  public enum intakestorage
  {
    nothing,
    cube,
    cone
  }

  public intakestorage m_currentStoredObject = intakestorage.nothing;

public final ClawModule m_ClawModule = new ClawModule(LiftConstants.kClawLeftID, 
                                                      LiftConstants.kClawRightID, 
                                                      LiftConstants.kClawRotationID, 
                                                      LiftConstants.kWristID, 
                                                      LiftConstants.kTopRollerID, 
                                                      LiftConstants.kBottomRollerID, 
                                                      LiftConstants.kConeDetector,
                                                      LiftConstants.kTouchSensor,
                                                      LiftConstants.kBlinkinID); 

public ClawSubsystem()
{
  //SmartDashboard.putNumber("rollerSpeed", LiftConstants.kintakeSpeedBottom);
  m_lifeExtendState = LiftSubsystem.extendPosition.intake;
}

//public RobotContainer m_RobotContainer = new RobotContainer();

  @Override
  public void periodic() 
  {

    SmartDashboard.putNumber("Cancoder readings: ", m_ClawModule.getwristEncoder().getPosition());

    //SmartDashboard.putBoolean("button pressed: ", m_ClawModule.getTouchDetector().get());
/* 
    m_ClawModule.getclawRotation().setNeutralMode(NeutralMode.Brake);

    TalonSRXConfiguration talonsrx = new TalonSRXConfiguration();
    m_ClawModule.getclawRotation().getAllConfigs(talonsrx);
   */ 

   // SmartDashboard.putBoolean("is Cone Beam Broken? = ",  m_ClawModule.getconeDetector().get());
    /* 
    ColorMatchResult match = m_ClawModule.getcolorMatch().matchClosestColor(m_ClawModule.getboxDetector().getColor());

      SmartDashboard.putNumber("color blue test: " , m_ClawModule.getboxDetector().getColor().blue);
      SmartDashboard.putNumber("color red test: " , m_ClawModule.getboxDetector().getColor().red);
      SmartDashboard.putNumber("color green test: " , m_ClawModule.getboxDetector().getColor().green);

      SmartDashboard.putBoolean("I See CUBE: ", match.color == ColorConstants.kPurpleTarget);
      SmartDashboard.putNumber("Confidence: ", match.confidence);

      SmartDashboard.putString("Whats in Intake? : ", (getIntakestorage() == intakestorage.nothing) ? "nothing" :
                                                           (getIntakestorage() == intakestorage.cube) ? "cube" : 
                                                          "cone"); // ? = if . . . . : = else 


      SmartDashboard.putNumber("wrist absolute angle : ", m_ClawModule.getwristEncoder().getAbsolutePosition());
    */
  }


  public void ClawDrive(boolean intakein, boolean expell, boolean clawtoIntake, boolean clawtoExpell, boolean isBlock, double heightOfLiftCurrent)  
  {  

    
    if (intakein == true)
    {
      intakeIn();
    }

    else if (expell == true)
    {
       intakeOut();
    }
    
    //the false here is backwards becuase it just is, it actually means that the button is pressed
   // else if (isBlock == false && heightOfLiftCurrent < m_LiftConstants.kExtendorPositionlow && m_LiftConstants.kExtendorPositionCone < heightOfLiftCurrent) {
     else if ( (m_lifeExtendState != extendPosition.intake) && (m_lifeExtendState != extendPosition.intookCone )) 
     {
      rollersInOnly();

     }
     

    //else if ( heightOfLiftCurrent < (m_LiftConstants.kExtendorPositionlow * LiftConstants.LIFT_COUNTS_PER_INCH))
    else{
      intakeOff();
    }

    if ( clawtoIntake )
    {
      clawToIntake();
    }

    else if (clawtoExpell)
    {
      clawToScore(extendPosition.high);
    }

    
  }


  //function to just run the rollers, made to hold cube in when needed
  public void rollersInOnly() {


    //keeps the rollers on at 20% power
    m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed / 4);
    m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeedBottom / 4);
    

    //turn claw motors off
    m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, 0.1);
    m_ClawModule.getclawRight().set(ControlMode.PercentOutput, 0.1);

  }
   
    /************************************************************************************** */
    /*This Function will turn on the claw and roller motors to intake a game piece          */
    //
    /************************************************************************************** */
    public void intakeIn() 
    {   
      

      //check the status of the cone detection beam (cone has been detected if False???)
      boolean m_coneDetectionBeam = m_ClawModule.getconeDetector().get();  
      boolean m_touchSensor = m_ClawModule.getTouchDetector().get();  
  
      // Check if the cone or block has been intook into the claw.  If not run motors to intake game piece
      //if (/*match.color != ColorConstants.kPurpleTarget || */ m_coneDetectionBeam == true /* ||  match.confidence <= ColorConstants.colorConfidenceTreshold */)
      if ( (m_coneDetectionBeam == true) && (m_touchSensor == true))
      {
        //intaking
       m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
       m_ClawModule.getclawRight().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
        
       m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
      // double speedBottom = SmartDashboard.getNumber("rollerSpeed", LiftConstants.kintakeSpeedBottom);
      // m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, speedBottom);

       m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeedBottom);
       m_currentStoredObject = intakestorage.nothing;

      }
      // If game piece is detected, stop motors
      else
      {
        //intook
        m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, 0);
        m_ClawModule.getclawRight().set(ControlMode.PercentOutput, 0);
  
        m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, 0);
        m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, 0);

        //based on if Cone or Cube is detected, update storage object to hold status.
        if ( m_coneDetectionBeam == false)
        {
          m_currentStoredObject = intakestorage.cone;
         
        }

        else 
        {
          m_currentStoredObject = intakestorage.cube;

        }
      }
  
    }

    /************************************************************************************** */
    /*This Function will turn on the claw and roller motors to eject a game piece          */
    //
    /************************************************************************************** */
    public void intakeOut() 
    {
  
      m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);
      m_ClawModule.getclawRight().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);

      m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);
      m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);

      m_currentStoredObject = intakestorage.nothing;
  
    }


    /************************************************************************************** */
    /*This Function will turn OFF the claw and roller motors when ejecting a game piece     */
    //
    /************************************************************************************** */
    public void intakeOff()
    {
        
      m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, 0);
      m_ClawModule.getclawRight().set(ControlMode.PercentOutput, 0);

      m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, 0);
      m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, 0);

    }

    public void setLiftState(LiftSubsystem.extendPosition state )
    {
      m_lifeExtendState = state;
    }
  
    /************************************************************************************** */
    /*Rotates the claw depending on what height your lift is at     */
    //
    /************************************************************************************** */
    public void clawToScore( extendPosition currentposition)
    {
      switch (currentposition)
      {
        case intake:
         m_ClawModule.getclawRotation().set(ControlMode.Position, LiftConstants.kWristToIntakePOS);
        break;

        case intookCone:
        m_ClawModule.getclawRotation().set(ControlMode.Position, LiftConstants.kWristToIntakePOS);
       break;

        case low:
          m_ClawModule.getclawRotation().set(ControlMode.Position, LiftConstants.kWristToScorePOSlow);
        break;

        case mid:
        m_ClawModule.getclawRotation().set(ControlMode.Position, LiftConstants.kWristToScorePOSmid);
        break;

        case high:
        m_ClawModule.getclawRotation().set(ControlMode.Position, LiftConstants.kWristToScorePOShigh);
        break;
      }
      

    
    }
  
    /************************************************************************************** */
    /*Sets the claw so that it will fit inside the intake     */
    //
    /************************************************************************************** */
    public void clawToIntake()
    {
   
      m_ClawModule.setClawSpeed(ClawConstants.kWristDownPOW);
      //m_ClawModule.setClawSpeed(ClawConstants.kWristDownPOW);
      m_ClawModule.getclawRotation().set(ControlMode.Position, LiftConstants.kWristToIntakePOS);
      
    }

    /************************************************************************************** */
    /*tells whether you are holding a cube, cone, or nothing     */
    //
    /************************************************************************************** */
    public intakestorage getIntakestorage()
    {
      return m_currentStoredObject;
    }

    public void setBlinkinto(double d)
    {
      m_ClawModule.getBlinkin().setSpeed(d);
    }

    


}
