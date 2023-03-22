package frc.robot.commands;
import javax.net.ssl.TrustManagerFactory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class balance extends CommandBase {

    int time;
    int timeMoving;
    double speed;//makes the speed that you want to set to the motors
    boolean parked;//determining if on ramp
    boolean start; //sees if you needed to start balancing yet
    private SlewRateLimiter m_SlewPitch;
    double initialPitch;
    boolean m_bFirst = true;

    private final DriveSubsystem m_driveTrain;
 
    //subsystem constructor
    public balance(DriveSubsystem subsystem) {
      m_driveTrain = subsystem;
      addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      SmartDashboard.putString("DriveDistance", "initialize");
      speed = 0;
      time = 0;  
      timeMoving = 0;
      start = false;
      parked = false;
      m_SlewPitch = new SlewRateLimiter(10);

      //m_driveTrain.m_gyro.setYaw(0);
      //initialPitch = m_driveTrain.m_gyro.getYaw();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { 

      if (m_bFirst == true)
      {
        initialPitch = m_driveTrain.m_gyro.getRoll();
        m_bFirst = false;
      }

      //sets the speed to what it needs to be
      if (balanceOnRamp() < 1){
        speed = balanceOnRamp();
      }

      //stops the robot from moving and finishs the code
      else if (balanceOnRamp() >= 1){
        m_driveTrain.setX(); 
        speed = 0;
        parked = true;
      }

      m_driveTrain.drive(
        //adds deadband (makes it so the controls aren't hyper sensitive)
        -MathUtil.applyDeadband(-speed, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
        false, false
      );
      
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      //once you've set the X position, the robot is parked and the code stops                                                                                                           congrats
      if (parked == true)
        return true;
      else
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
      return false;
    }

  //method that actually does the balancing
  public double balanceOnRamp() {
    

   double sped;//variable for robotspeed
   double returnSpeed = 0;//is the speed returned to then use

   //double pitch = m_SlewPitch.calculate(m_driveTrain.m_gyro.getYaw() - initialPitch);
   double pitch = m_driveTrain.m_gyro.getRoll() - initialPitch;
   SmartDashboard.putNumber("Pitch", pitch);
  SmartDashboard.putNumber("ReturnSpeed", returnSpeed);
   SmartDashboard.putNumber("Time", time);
   SmartDashboard.putBoolean("Start", start);
   SmartDashboard.putNumber("Initial Pitch", m_driveTrain.m_gyro.getRoll());
   
   
   //keeps running until it's balanced for a set amount of time
  if(time <= 500){
    //if (true){
   
    // says to keep going until you've gotten onto the ramp
    if(Math.abs(pitch) < 4 && start == false){
     returnSpeed = 0.6;
     //return returnSpeed;
    } 
    else{
      start = true;
    }

    //sets the speed relative to the degree it's at
    sped = Math.abs(pitch/50);

    if (timeMoving < 250){
      //makes sure the motors use enough power to get up
      if(sped < 0.1){
        sped = 0.1;
        timeMoving ++;
      } 

      //makes sure it doesn't give it to much power
      else if (sped > 0.1){
        sped = 0.1;
        timeMoving ++;
      }
    }
    else {
      //makes sure the motors use enough power to get up
      if(sped < 0.50){
        sped = 0;
     } 

     //makes sure it doesn't give it to much power
     else if (sped > 0.50){
       sped = 0;
     }
   }
    //if tilted frontside, tells it to go foward
    if(pitch > 2 && start == true){
     //drive forward
      returnSpeed = sped;
     time = 0; //resets the clock
    } 

    //if tilted backside, tells it to go backwards
    else if(pitch  < -2 && start == true){
      //drive back
      returnSpeed = -sped;
      time = 0; //resets the clock
    }

    //if flat, tells it to keep track of how long it's been flat and stops the motors
    else if(start == true){
      returnSpeed = 0;//stay still
      time ++; //adds to the ticker to keep track of time put in the end of call
      //timeMoving = 0;
    }
   }

  SmartDashboard.putNumber("Pitch", pitch);
  SmartDashboard.putNumber("ReturnSpeed", returnSpeed);
   SmartDashboard.putNumber("Time", time);
   SmartDashboard.putBoolean("Start", start);
   SmartDashboard.putNumber("Initial Pitch", m_driveTrain.m_gyro.getPitch());
   /* 
   //stops the code from looping
   if(time >= 2000){
     returnSpeed = 100; //makes the speed high so that it ends
   }
   */
   return returnSpeed;
  
 }
}