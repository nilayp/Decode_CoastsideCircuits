package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccAllianceChooser {

    public enum Alliance {
        RED,
        BLUE
    }

    public Alliance alliance = Alliance.RED;

    public void init() {

    }

    public void init_loop(Gamepad gamepad, Telemetry telemetry, ccLED left, ccLED right) {
        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad.circle) {
            alliance = Alliance.RED;
        } else if (gamepad.square) {
            alliance = Alliance.BLUE;
        }

        // LED 1 is used to indicate whether the robot is positioned
        // for the red goal or the blue goal.
        // RED LED is for RED GOAL.
        // GREEN LED BLUE GOAL.

        if (alliance == Alliance.RED) {
            right.setRedLed();
            left.setRedLed();
        } else {
            right.setGreenLed();
            left.setGreenLed();
        }

        telemetry.addData("Press SQUARE", "for BLUE");
        telemetry.addData("Press CIRCLE", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }
}
