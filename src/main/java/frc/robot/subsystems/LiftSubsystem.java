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
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.LiftConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClawSubsystem;



public class LiftSubsystem extends SubsystemBase {

//enumeraion for the claw position
public static enum extendPosition
{
  intake,
  intookCone,
  low,
  mid,
  high
}

//creats member variable for the claw and keeps track of claw postion (defaults to intake)
public extendPosition m_currentPOSofLift = extendPosition.intake;

public boolean ExtendorInMotion; //tells if the lift is currently moving

public LiftModule m_liftModule;
private ClawSubsystem m_clawSubsystem;

public LiftSubsystem(ClawSubsystem claw)
{
    m_clawSubsystem = claw;

    //creats the lift module object
    m_liftModule = new LiftModule(LiftConstants.kExtendorRightCanID, LiftConstants.kExtendorLeftCanID, LiftConstants.kLiftLimitSwitchLeft, LiftConstants.kLiftLimitSwitchRight, LiftConstants.kPneumaticCANID , LiftConstants.kPneumaticPort );

}

                                                                                    
//subsystem overide that is called every 20 milliseconds
public void periodic() {
  double counts = m_liftModule.getExtendorRight().getSelectedSensorPosition();
  SmartDashboard.putNumber("power of lift motor: ", m_liftModule.getExtendorRight().getMotorOutputPercent());
  SmartDashboard.putNumber("current position of extendors in counts: ", counts);
  SmartDashboard.putNumber("current position of extendors in inch: " ,counts/ LiftConstants.LIFT_COUNTS_PER_INCH);

  SmartDashboard.putNumber("current power of lift motor: ", m_liftModule.getExtendorRight().get());

  SmartDashboard.putBoolean("Digital Pressure Switch", m_liftModule.getPneumaticHub().getPressureSwitch());
  SmartDashboard.putBoolean("Digital Compressor Switch", m_liftModule.getPneumaticHub().getCompressor());
}

    /************************************************************************************** */
    /*This Function will command the lift to a certain position based of controller input     */
    //extendor Speed = the speed you want to motors to run 
    //extendHigh = boolean for if you are extending to the high node
    //extendMid = boolean for if you are extending to the medium node
    //extendLow = boolean for if you are extending to the low node
    //extendRest = boolean for if it is at the rest/intake position
    //extendPneumatic = boonean for extending the pneumatic cone tipper
    /************************************************************************************** */
  public void liftDrive(double extendorSpeed, boolean extendHigh, boolean extendMid, boolean extendLow, boolean extendRest,boolean extendPnematic) {

    
      
        //if extendLow button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position
        if (extendLow  && ( m_currentPOSofLift != extendPosition.low ) )
        {
        // double currentPositionRight = m_liftModule.getExtendorRight().getSelectedSensorPosition(0);
          double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionlow);
          SmartDashboard.putNumber("target Position", targetPositionRotationsRight);
          
          m_clawSubsystem.m_ClawModule.setClawSpeed(ClawConstants.kWristUpPOW);
          m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .8;

          m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
          m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());

          m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
          SmartDashboard.putString("ButtonPress", "ExtendLow");

          m_currentPOSofLift = extendPosition.low; //keeps track of current position once the action is finished
        }

        //if extendMid button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position
        else if (extendMid  && ( m_currentPOSofLift != extendPosition.mid ))
        {

        // double currentPositionRight = m_liftModule.getExtendorRight().getSelectedSensorPosition(0);
          double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionmid);
          SmartDashboard.putNumber("target Position", targetPositionRotationsRight);

          m_clawSubsystem.m_ClawModule.setClawSpeed(ClawConstants.kWristUpPOW);
          m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .6;

          m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
          m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());
          
          m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
          SmartDashboard.putString("ButtonPress", "extendMid");

          m_currentPOSofLift = extendPosition.mid;
        }

        //if extendHigh button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position
        else if (extendHigh && ( m_currentPOSofLift != extendPosition.high ))
        {
          double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionhigh);
          SmartDashboard.putNumber("target Position", targetPositionRotationsRight);

          m_clawSubsystem.m_ClawModule.setClawSpeed(1);
          m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .6;

          m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
          m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());
          
          m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
          SmartDashboard.putString("ButtonPress", "extendHigh");
          m_currentPOSofLift = extendPosition.high;

        }
/* 
    //if there is a cube use this drop code
    else if (m_clawSubsystem.m_ClawModule.getTouchDetector().get() == false)
    {
            //if extendLow button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position
        if (extendLow  && ( m_currentPOSofLift != extendPosition.low ) )
        {
        // double currentPositionRight = m_liftModule.getExtendorRight().getSelectedSensorPosition(0);
          double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionlow);
          SmartDashboard.putNumber("target Position", targetPositionRotationsRight);
          
          m_clawSubsystem.m_ClawModule.setClawSpeed(ClawConstants.kWristUpPOW);
          m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .8;

          m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
          m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());

          m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
          SmartDashboard.putString("ButtonPress", "ExtendLow");

          m_currentPOSofLift = extendPosition.low; //keeps track of current position once the action is finished
        }

        //if extendMid button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position
        else if (extendMid  && ( m_currentPOSofLift != extendPosition.mid ))
        {

        // double currentPositionRight = m_liftModule.getExtendorRight().getSelectedSensorPosition(0);
          double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionmid);
          SmartDashboard.putNumber("target Position", targetPositionRotationsRight);

          m_clawSubsystem.m_ClawModule.setClawSpeed(ClawConstants.kWristUpPOW);
          m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .6;

          m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
          m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());
          
          m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
          SmartDashboard.putString("ButtonPress", "extendMid");

          m_currentPOSofLift = extendPosition.mid;
        }

        //if extendHigh button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position
        else if (extendHigh && ( m_currentPOSofLift != extendPosition.high ))
        {
          double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionhigh);
          SmartDashboard.putNumber("target Position", targetPositionRotationsRight);

          m_clawSubsystem.m_ClawModule.setClawSpeed(1);
          m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .6;

          m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
          m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());
          
          m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
          SmartDashboard.putString("ButtonPress", "extendHigh");
          m_currentPOSofLift = extendPosition.high;

        }



    }

    */

    

    //if extendRest button = true, use constant to calculate the encoder position, then make the lift module go to said calculated position (rest/intakle pos)
    else if (extendRest && ( m_currentPOSofLift != extendPosition.intake )
    && (m_clawSubsystem.m_ClawModule.getconeDetector().get() == true && m_clawSubsystem.m_ClawModule.getTouchDetector().get() == true))
    {
      double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionReset);
      SmartDashboard.putNumber("target Position", targetPositionRotationsRight);

      m_clawSubsystem.m_ClawModule.setClawSpeed(ClawConstants.kWristDownPOW);
      m_liftModule.getconfigs().slot0.closedLoopPeakOutput = .6;

      m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
      m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());
      
      m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);
      SmartDashboard.putString("ButtonPress", "extendRest");
      m_currentPOSofLift = extendPosition.intake;

    }
    
    else if ((m_clawSubsystem.m_ClawModule.getconeDetector().get() == false) && ( m_currentPOSofLift == extendPosition.intake ))
  
    //else if ((m_clawSubsystem.m_currentStoredObject == m_clawSubsystem.m_currentStoredObject.cone) && ( m_currentPOSofLift == extendPosition.intake ))
    {
      double targetPositionRotationsRight = (LiftConstants.LIFT_COUNTS_PER_INCH * LiftConstants.kExtendorPositionCone);
      SmartDashboard.putNumber("target Position", targetPositionRotationsRight);

      m_liftModule.getExtendorRight().configAllSettings(m_liftModule.getconfigs());
      m_liftModule.getExtendorLeft().configAllSettings(m_liftModule.getconfigs());
      
      m_liftModule.getExtendorRight().set(ControlMode.Position, targetPositionRotationsRight);

      m_currentPOSofLift = m_currentPOSofLift.intookCone;
    }

    //makes sure the tipper goes back to its inactive position
    else if (extendorSpeed > 0.05 || extendorSpeed < -0.05) 
    {
     // m_liftModule.getconeDeporter().set(false);
      double liftYstick = -extendorSpeed;
      
      //takes joystick and limits the lift speed to 30%
      if (liftYstick >= 0.3)
      {
        liftYstick = 0.3;
      }

      //detects if the lift is at the bottom limit switch, if it is not, it lets you continue moving the lift down
      else if ((liftYstick <= -0.3)  && (m_liftModule.getLiftLimitSwitchLeft().get() == true) && (m_liftModule.getLiftLimitSwitchRight().get() == true))
      {
        liftYstick = -0.3;
      }

      //if it is at the limit, it sets the lift speed to 0 so it cannot continue moving downward and commit die
      else
      {
        liftYstick = 0;
        
        
      }

      //applys power to the lift motors depending on the results of the if block above


      m_liftModule.getExtendorRight().set(ControlMode.PercentOutput, liftYstick);
      
    
    }
    else if (m_liftModule.getPIDError() < LiftConstants.kAllowableError){ //this resets the integral so that it hopefully doesn't overextend and then underextend
      m_liftModule.resetIntegral();
    }
///////NEW
     //if extendPneumatic button = true, extend the pneumatic cone tipper to pneumaticaly tip cones so they can be intaked
    //if the current postion of the lift is in the intake state then the pnuematics will be allowed to fire. 
     if (extendPnematic && (m_currentPOSofLift == extendPosition.intake)){
      m_liftModule.getconeDeporter().set(true);
    
    }

    else 
    {
      m_liftModule.getconeDeporter().set(false);
    }
///////NEW

    if ((m_currentPOSofLift == extendPosition.intake) && (Math.abs(m_liftModule.getExtendorRight().getSelectedSensorPosition()) < LiftConstants.LiftdownEnough))
    {
      m_clawSubsystem.m_ClawModule.getclawRotation().set(ControlMode.PercentOutput, 0);
    }

    //if the limit switch at the bottom is hit, resets the lift encoder to 0 so its position is accurate
    if ((m_liftModule.getLiftLimitSwitchLeft().get() == false) || (m_liftModule.getLiftLimitSwitchRight().get() == false))
    {
      m_liftModule.getExtendorRight().setSelectedSensorPosition(0);
      m_clawSubsystem.m_ClawModule.getclawRotation().set(ControlMode.PercentOutput, 0);
      m_currentPOSofLift = m_currentPOSofLift.intake;
    }

  }



  public void extend() {
    // this method will be called to extend the pneumatic bar
    m_liftModule.getconeDeporter().set(true);

}

public void retract() {
    // this method will be called to retract the pneumatic bar
   m_liftModule.getconeDeporter().set(false);

}



//returns where the lift is at
  public extendPosition getCurrentExtendPosition(){ 
      return m_currentPOSofLift;
  }

  //returns all the lift motors and the tipper
  public LiftModule getLiftModule(){
    return m_liftModule;
  }

}
