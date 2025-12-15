package org.firstinspires.ftc.teamcode.mechanisms;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccPositionalServo {

    private Servo servo = null;

    private PwmControl.PwmRange positional_pwmRange;

    double targetDeg = 0.0;
    double currentDeg = 0.0;
    double maxDegrees = 0.0;
    double maxDegreePerStep = 0.0;

    public void init(HardwareMap map, String deviceName, double md, double mdps, Telemetry telemetry) {
        servo = map.get(Servo.class, deviceName);

        // Match Stingray’s full control band (spec is ~500–2500 µs).
        // This lets 0.0 -> 500 µs and 1.0 -> 2500 µs for full 200°.

        if (servo instanceof PwmControl) {
            positional_pwmRange = new PwmControl.PwmRange(500, 2500);
            ((PwmControl) servo).setPwmRange(positional_pwmRange);
        }

        // Start from wherever at the center position)
        maxDegrees = md;
        maxDegreePerStep = mdps;
        targetDeg = maxDegrees / 2;
        currentDeg = targetDeg;
        servo.setPosition(degToPos(currentDeg, maxDegrees));

        telemetry.addData("Target Degree", targetDeg);
        telemetry.addData("Max Degrees", maxDegrees);
        telemetry.addData("Max Degrees Per Step", maxDegreePerStep);

        telemetry.addLine("Dpad Right = set turret in middle position");
        telemetry.addLine("Right Trigger = move turret to the right");
        telemetry.addLine("Left Trigger = move turret to the left");
    }

    public void teleOpLoop(Gamepad gamepad, Telemetry telemetry){

        // Dpad-right moves turret to the right
        if (gamepad.dpadRightWasPressed()) {
            targetDeg += maxDegreePerStep;
        }
        // Dpad-left moves turret to the left
        if (gamepad.dpadLeftWasPressed()) {
            targetDeg -= maxDegreePerStep;
        }
        // Share button resets turntable to the middle allowable position
        if (gamepad.share) {
            targetDeg = maxDegrees / 2;
        }

        // Clamp into the physical range to prevent the servo from trying to move to an
        // that impossible position.
        targetDeg = Range.clip(targetDeg, 0, maxDegrees);

        // Optional slew (smoothness) for turntable & ramp
        // This assures that in each loop cycle, the servo only moves by the
        // MAX_DEG_PER_STEP constant and doesn't jerk.
        if (maxDegreePerStep > 0) {
            double error = targetDeg - currentDeg;
            double step = Range.clip(error, -maxDegreePerStep, maxDegreePerStep);
            currentDeg += step;
        } else {
            currentDeg = targetDeg;
        }

        // Command the servos
        servo.setPosition(degToPos(currentDeg, maxDegrees));

        // Telemetry for both servos
        telemetry.addData("Turntable Target (deg)", "%.1f", targetDeg);
        telemetry.addData("Turntable Command (deg)", "%.1f", currentDeg);
        telemetry.addData("Turntable Position (0..1)", "%.3f", degToPos(currentDeg, maxDegrees));

        sleep(20); // ~50 Hz loop
    }

    private double degToPos(double deg, double maxDegrees) {
        // Map 0..200° -> 0.0..1.0
        double pos = deg / maxDegrees;
        return Range.clip(pos, 0.0, 1.0);
    }
}
