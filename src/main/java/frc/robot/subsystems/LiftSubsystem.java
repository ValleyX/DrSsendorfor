// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
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
import frc.robot.Constants.LiftConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

public enum extendPosition
{
  low,
  mid,
  high
}


private final LiftModule m_liftModule = new LiftModule(LiftConstants.kExtendorRightCanID, LiftConstants.kExtendorLeftCanID, LiftConstants.kTopRoller,
LiftConstants.kBottomRoller, LiftConstants.kClawLeft, LiftConstants.kClawRight, LiftConstants.kClawRotation, LiftConstants.kLiftExtended,
LiftConstants.kLiftContracted);

  @Override
  public void periodic() {
      //add needed telemetry stuff here
      }


  public void liftDrive(double extendorSpeed, int rotDOWN, int rotUP, boolean extendHigh, boolean extendMid, boolean extendLow) {
    
    extendorSpeed = 0.1;
    SmartDashboard.putNumber("XSpeed", extendorSpeed);

    //m_liftModule.getExtendorRight().set(extendorSpeed); 
    //m_liftModule.getExtendorLeft().set(extendorSpeed);

    SmartDashboard.putNumber("current Position", m_liftModule.getExtendorRight().getSelectedSensorPosition());
    if (extendLow)
    {
      //SetPercentOutput(extendorSpeed, LiftConstants.kTimeoutMs);
        

     // double currentPositionRight = m_liftModule.getExtendorRight().getSelectedSensorPosition(0);
      double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionlow);
      SmartDashboard.putNumber("target Position", targetPositionRotationsRight);
      m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
      SmartDashboard.putString("ButtonPress", "ExtendLow");
    }

    else if (extendMid)
    {
      //SetPercentOutput(extendorSpeed, LiftConstants.kTimeoutMs);
        

     // double currentPositionRight = m_liftModule.getExtendorRight().getSelectedSensorPosition(0);
      double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionmid);
      SmartDashboard.putNumber("target Position", targetPositionRotationsRight);
      m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
      SmartDashboard.putString("ButtonPress", "extendMid");
  
    }
    else
    {
      SmartDashboard.putString("ButtonPress", "None");
    }
    
/* 
    switch (heightOfLift) {
      case low: 

      break;

      case mid:

      break;

      case high:
    }
    */
/* 
    if(rotDOWN)
    {
    m_liftModule.getClawRotation().set(ControlMode.Position, LiftConstants.kClawRotationDOWN);
    }

    if (rotUP)
    {
    m_liftModule.getClawRotation().set(ControlMode.Position, LiftConstants.kClawRotationUP);
    }
*/
    


  }
  



}
