package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccContinuousRotationServo {

    private CRServo servo = null;

    public void init(HardwareMap map, String deviceName, Telemetry telemetry) {
        servo = map.get(CRServo.class, deviceName);

        telemetry.addLine("Dpad Down = spin intake forward");
        telemetry.addLine("Dpad Up = spin intake backward");
    }

    public void teleOpLoop(Gamepad gamepad, Telemetry telemetry){
        // Triggers give values 0.0 → 1.0
        boolean forward = gamepad.dpad_down;
        boolean backward = gamepad.dpad_up;

        double power = 0.0;

        if (forward) {
            power = 1.0;
        } else if (backward) {
            power = -1.0;
        }

        servo.setPower(power);

        telemetry.addData("Intake Power", "%.2f", power);
    }

}
