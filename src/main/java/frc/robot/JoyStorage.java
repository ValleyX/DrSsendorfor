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
        public boolean buttonX; //lift  reset ;
        public boolean buttonR3;
        public boolean buttonL3;
        public boolean buttonBack;

        public boolean bumperLeftExpell;
        public boolean bumperRightIntake;

        public boolean driverButtonR3;
        public boolean driverButtonL3;
        



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
        boolean buttonR3,
        boolean buttonL3,
        boolean bumperRightIntake,
        boolean bumperLeftExpell,
        boolean buttonBack,
        boolean driverButtonL3,
        boolean driverButtonR3) {
        this.leftdriveYstick = leftdriveYstick;
        this.leftdriveXStick = leftdriveXStick;
        this.rightdriveXstick = rightdriveXstick;
        this.manipulatorLeftYstick = manipulatorLeftYstick;
        this.buttonA = buttonA;
        this.buttonB = buttonB;
        this.buttonY = buttonY;
        this.buttonX = buttonX;
        this.buttonR3 = buttonR3;
        this.buttonL3 = buttonL3;
        this.buttonBack = buttonBack;
        this.bumperLeftExpell = bumperLeftExpell;
        this.bumperRightIntake = bumperRightIntake;
        this.driverButtonL3 = driverButtonL3;
        this.driverButtonR3 = driverButtonR3;

    }

    // getters and setters, toString() .... (omitted for brevity)
}