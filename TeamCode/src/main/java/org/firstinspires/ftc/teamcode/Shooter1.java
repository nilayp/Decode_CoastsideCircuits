package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;   // for PWM range
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Shooter1")
public class Shooter1 extends LinearOpMode {

    // ---- CONFIG ----
    // The Stingray-9 in feedback mode has ~200° total travel.
    private static final double MAX_DEGREES = 200.0;
    // Slew: max degrees per loop step (set to 0 to disable smoothing)
    private static final double MAX_DEG_PER_STEP = 4.0;

    private Servo turntable;
    private PwmControl.PwmRange pwmRange;

    private double targetDeg = 0.0;     // where we want to go
    private double currentDeg = 0.0;    // our internal estimate of where we’ve commanded

    @Override
    public void runOpMode() throws InterruptedException {
        turntable = hardwareMap.get(Servo.class, "turntable");

        // Match Stingray’s full control band (spec is ~500–2500 µs).
        // This lets 0.0 -> 500 µs and 1.0 -> 2500 µs for full 200°.
        if (turntable instanceof PwmControl) {
            pwmRange = new PwmControl.PwmRange(500, 2500);
            ((PwmControl) turntable).setPwmRange(pwmRange);
        }

        // Start from wherever you want (0° = one mechanical end)
        targetDeg = 0.0;
        currentDeg = targetDeg;
        turntable.setPosition(degToPos(currentDeg));

        telemetry.addLine("Stingray-9 Plate Control (Feedback Mode)");
        telemetry.addLine("PS4: triangle=0°, square=90°, circle=180°");
        telemetry.addLine("Dpad Up/Down = ±5° | Left/Right = ±15°");
        telemetry.addLine("Hold Cross (X) to center to 90°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Presets (PS4 aliases work in FTC SDK) ---
            if (gamepad1.triangle) targetDeg = 0;       // 0°
            if (gamepad1.square)   targetDeg = 90;      // 90°
            if (gamepad1.circle)   targetDeg = 180;     // 180°
            if (gamepad1.cross)    targetDeg = 90;      // quick center

            // --- Nudge controls ---
            if (gamepad1.dpad_up)    targetDeg += 5;
            if (gamepad1.dpad_down)  targetDeg -= 5;
            if (gamepad1.dpad_right) targetDeg += 15;
            if (gamepad1.dpad_left)  targetDeg -= 15;

            // Clamp into the physical range
            targetDeg = Range.clip(targetDeg, 0, MAX_DEGREES);

            // Optional slew (smoothness)
            if (MAX_DEG_PER_STEP > 0) {
                double error = targetDeg - currentDeg;
                double step = Range.clip(error, -MAX_DEG_PER_STEP, MAX_DEG_PER_STEP);
                currentDeg += step;
            } else {
                currentDeg = targetDeg;
            }

            // Command the servo
            turntable.setPosition(degToPos(currentDeg));

            telemetry.addData("Target (deg)", "%.1f", targetDeg);
            telemetry.addData("Command (deg)", "%.1f", currentDeg);
            telemetry.addData("Position (0..1)", "%.3f", degToPos(currentDeg));
            telemetry.update();

            sleep(20); // ~50 Hz loop
        }
    }

    private double degToPos(double deg) {
        // Map 0..200° -> 0.0..1.0
        double pos = deg / MAX_DEGREES;
        return Range.clip(pos, 0.0, 1.0);
    }
}