package frc.robot;

import java.io.Serializable;

public class JoyStorage implements Serializable {

        public double leftdriveYstick; //swerve forwards and backwards
        public double leftdriveXStick; //swerve left and right
        public double rightdriveXstick; //swerve rotation
        public double manipulatorLeftYstick;

        public boolean buttonA; // lift to low
        public boolean buttonB; // lift to mid
        public boolean buttonY; // lift to high
        public boolean buttonX; //lift  reset 

        public boolean bumperLeftExpell;
        public boolean bumperRightIntake;



    public JoyStorage() {
    }

    public JoyStorage(
        double leftdriveYstick,
        double leftdriveXStick,
        double rightdriveXstick,
        double manipulatorLeftYstick,
        boolean buttonA,
        boolean buttonB,
        boolean buttonY,
        boolean buttonX,
        boolean bumperLeftExpell,
        boolean bumperRightIntake) {
        this.leftdriveYstick = leftdriveYstick;
        this.leftdriveXStick = leftdriveXStick;
        this.rightdriveXstick = rightdriveXstick;
        this.manipulatorLeftYstick = manipulatorLeftYstick;
        this.buttonA = buttonA;
        this.buttonB = buttonB;
        this.buttonY = buttonY;
        this.buttonX = buttonX;
        this.bumperLeftExpell = bumperLeftExpell;
        this.bumperRightIntake = bumperRightIntake;

    }

    // getters and setters, toString() .... (omitted for brevity)
}