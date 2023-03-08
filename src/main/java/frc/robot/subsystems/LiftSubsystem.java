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


public final LiftModule m_liftModule = new LiftModule(LiftConstants.kExtendorRightCanID, LiftConstants.kExtendorLeftCanID);

  @Override
  public void periodic() {
      //add needed telemetry stuff here
      }


  public void liftDrive(double extendorSpeed, boolean extendHigh, boolean extendMid, boolean extendLow, boolean extendRest) {
    
    SmartDashboard.putNumber("XSpeed", extendorSpeed);

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
    else if (extendHigh)
    {

      double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionhigh);
      SmartDashboard.putNumber("target Position", targetPositionRotationsRight);
      m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
      SmartDashboard.putString("ButtonPress", "extendMid");
  
    }

    else if (extendRest)
    {
      double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionReset);
      SmartDashboard.putNumber("target Position", targetPositionRotationsRight);
      m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
      SmartDashboard.putString("ButtonPress", "extendMid");
    }
    
    else 
    {
      
      double liftYstick = -extendorSpeed;

      if (liftYstick >= 0.3)
      {
        liftYstick = 0.3;
      }

      else if (liftYstick <= -0.3)
      {
        liftYstick = -0.3;
      }

      else if (Math.abs(liftYstick) < 0.1)
      {
        liftYstick = 0;
      }

      m_liftModule.getExtendorRight().set(ControlMode.PercentOutput, liftYstick);

    }
    
  }

}
