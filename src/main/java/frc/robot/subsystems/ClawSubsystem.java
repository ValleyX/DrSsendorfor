// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.LiftConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

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
                                                      LiftConstants.kConeDetector); 

  @Override
  public void periodic() 
  {
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
    
  }


  public void ClawDrive(boolean intakein, boolean expell)  
  {  
    if (intakein == true)
    {
      intakeIn();
    }

    else if (expell == true)
    {
      intakeOut();
    }

    else 
    {

    }
    


  }
   
    public void intakeIn() 
    {
    

      ColorMatchResult match = m_ClawModule.getcolorMatch().matchClosestColor(m_ClawModule.getboxDetector().getColor());

      
  
     boolean m_coneDetectionBeam = m_ClawModule.getconeDetector().get();
  
  
      if (match.color != ColorConstants.kPurpleTarget || m_coneDetectionBeam == false || match.confidence <= ColorConstants.colorConfidenceTreshold )
      {
        /*intaking
       m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
       m_ClawModule.getclawRight().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
        
       m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
       m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, LiftConstants.kintakeSpeed);
*/
       m_currentStoredObject = intakestorage.nothing;

      }
  
      else
      {
        /*intook
        m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, 0);
        m_ClawModule.getclawRight().set(ControlMode.PercentOutput, 0);
  
        m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, 0);
        m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, 0);
*/
        if ( m_coneDetectionBeam == true)
        {
          m_currentStoredObject = intakestorage.cone;
        }

        else 
        {
          m_currentStoredObject = intakestorage.cube;
        }
      }
  
    }

    public void intakeOut() 
    {
  /* 
      m_ClawModule.getclawLeft().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);
      m_ClawModule.getclawRight().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);

      m_ClawModule.gettopRoller().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);
      m_ClawModule.getbottomRoller().set(ControlMode.PercentOutput, LiftConstants.kexpellSpeed);
*/
      m_currentStoredObject = intakestorage.nothing;
  
    }
  
    public void clawToScore()
    {
  
    
    }
  
    public void clawToIntake()
    {
  
  
    }


    public intakestorage getIntakestorage()
    {
      return m_currentStoredObject;
    }


}
