
package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;



public class pneumaticConeTipper extends SubsystemBase {
   
    private Solenoid coneDeporter;

 
    public pneumaticConeTipper() {
        coneDeporter = new Solenoid(LiftConstants.kPneumaticCANID , PneumaticsModuleType.CTREPCM, LiftConstants.kPneumaticPort);
        addChild("coneDeporter", coneDeporter);
     }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public Solenoid getconeDeporter() {
            return coneDeporter;
    }

    public void extend() {
        // this method will be called to extend the pneumatic bar
        coneDeporter.set(on: true);

    }

    public void retract() {
        // this method will be called to retract the pneumatic bar
        coneDeporter.set(on: false)

    }
}




