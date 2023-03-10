// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    //public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxSpeedMetersPerSecond = 20;
   // public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularSpeed = 12 * Math.PI; // radians per second // how fast robot turns

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(31);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    /* 
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    */

    /**
     * to find the offset open the "REV Hardware Client" application and plug in computer to the Power Distribution hub
     * then look at the SparkMAX absolute encoder that is related to the turn motor that you want to offset
     * square the wheel using provided bracket 
     * if the bracket alligned wheel is facing forward write the number of the absolute value here
     * if the bracket allignedd wheel is facing left, write the number of the absolute value but minus PI/2 off
     */
    public static final double kFrontLeftChassisAngularOffset = 1.600 - (Math.PI/2);
    public static final double kFrontRightChassisAngularOffset = 4.260;
    public static final double kBackLeftChassisAngularOffset = 3.145;
    public static final double kBackRightChassisAngularOffset = 5.922 + (Math.PI/2);


    // Drivetrain Talon FX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 7;

    // Drivetrain SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 3;

    public static final boolean kGyroReversed = false;


  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; //3 inches
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kCountsPerRev = 2048;
    public static final double kSecIn100MS = 0.1;
    public static final double k100MSinSec = 100;
    
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class LiftConstants {
    // Lift Talon FX CAN IDs
    public static final int kExtendorRightCanID = 10;
    public static final int kExtendorLeftCanID = 11;

    // Lift Talon SRX CAN IDs
    public static final int kTopRollerID = 12;
    public static final int kBottomRollerID = 13; 
    
    // Claw Talon SRX CAN IDs
    public static final int kClawLeftID = 14; 
    public static final int kClawRightID = 15;
    public static final int kClawRotationID = 16; 

    //Claw CANCoderID
    public static final int kWristID = 19; //cancoder

    // Digital input IDS
    public static final int kConeDetector = 1;

    // Positions 
    public static final int kWristToIntakePOS = 0;
    public static final double kWristToScorePOS = 52 * (4096/360);

    public static final int kPOIRotationDOWN = 180;
    public static final int kPOIRotationUP = 0;

    public static final int kExtendorPositionReset = 0;
    public static final int kExtendorPositionlow = 4; 
    public static final int kExtendorPositionmid = 32; //65
    public static final int kExtendorPositionhigh = 53; //106

    public static final double kintakeSpeed = 0.5;
    public static final double kexpellSpeed = -0.5;



    public static final double LIFT_COUNTS_PER_MOTOR_REV = 2048;    //  AndyMark Motor Encoder
    public static final double LIFT_DRIVE_GEAR_REDUCTION = 35;     // This is < 1.0 if geared UP
    public static final double LIFT_ONE_MOTOR_COUNT = LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION;
    public static final double LIFT_MOVEMENT_RATIO = 3;
    public static final double LIFT_DIAMETER_OF_PULLEY = 1.3;
    public static final double LIFT_Distance_in_one_rev = LIFT_DIAMETER_OF_PULLEY * Math.PI; //in
    public static final double LIFT_COUNTS_PER_INCH = (LIFT_ONE_MOTOR_COUNT / LIFT_Distance_in_one_rev) / LIFT_MOVEMENT_RATIO;  //TODO determine// in class

    public static final double kTimeoutMs = 100;

    // LIFT PID
    public static final double kLIFTP = 0.018;
    public static final double kLIFTI = 0.000025;
    public static final double kAllowableError = 200;

  }

  public static final class ColorConstants {

    public final static Color kPurpleTarget = new Color(0.211, 0.332, 0.456);
    public final static double colorConfidenceTreshold = 0.94;

  }

  public static final class ClawConstants {
    public final static double kClawP = 0.8;
    public final static double kClawD = 0;
    public final static double kClawI = 0.0008;
    public final static double allowableClawError = 0;
  }



  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;

    public static final int KManipulatorControllerPort = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3 ;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3  ;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =  Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 7000; //5676
  }
}
