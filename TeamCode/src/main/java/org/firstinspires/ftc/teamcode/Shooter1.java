package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;   // for PWM range
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name="Shooter1")
public class Shooter1 extends LinearOpMode {

    // ---- CONFIG ----
    // The Stingray-9 in feedback mode has ~200° total travel.
    private static final double TURNTABLE_MAX_DEGREES = 200.0;
    // Slew: max degrees per loop step (set to 0 to disable smoothing)
    private static final double TURNTABLE_MAX_DEG_PER_STEP = 4.0;

    private Servo turntable;
    private PwmControl.PwmRange turntable_pwmRange;

    private double turntable_targetDeg = 0.0;     // where we want to go
    private double turntable_currentDeg = 0.0;    // our internal estimate of where we’ve commanded

@Override
    public void runOpMode() throws InterruptedException {
        turntable = hardwareMap.get(Servo.class, "potato");

        // Match Stingray’s full control band (spec is ~500–2500 µs).
        // This lets 0.0 -> 500 µs and 1.0 -> 2500 µs for full 200°.
        if (turntable instanceof PwmControl) {
            turntable_pwmRange = new PwmControl.PwmRange(500, 2500);
            ((PwmControl) turntable).setPwmRange(turntable_pwmRange);
        }

        // Start from wherever at the center position)
        turntable_targetDeg = TURNTABLE_MAX_DEGREES / 2;
        turntable_currentDeg = turntable_targetDeg;
        turntable.setPosition(degToPos(turntable_currentDeg, TURNTABLE_MAX_DEGREES));

        waitForStart();

        while (opModeIsActive()) {
            // --- Incremental Stick Controls ---
            // Left stick X controls turntable movement rate
            double leftStickX = gamepad1.left_stick_x;
            // Move turntable based on stick input (rate control, not position control)
            turntable_targetDeg += leftStickX * TURNTABLE_MAX_DEG_PER_STEP; // Adjust RAMP_MAX_DEG_PER_STEP to change movement speed
            
            // Right stick Y controls ramp movement rate
            // Note: gamepad Y is inverted (up = -1, down = +1), so we invert it
            double rightStickY = -gamepad1.right_stick_y;

            // --- Button Overrides ---
            // Dpad-up resets turntable to 100°, or the middle allowable position
            if (gamepad1.dpad_up) {
                turntable_targetDeg = TURNTABLE_MAX_DEGREES / 2;
            }

            // Clamp into the physical range to prevent the servo from trying to move to an
            // that impossible position.
            turntable_targetDeg = Range.clip(turntable_targetDeg, 0, TURNTABLE_MAX_DEGREES);

            // Optional slew (smoothness) for turntable & ramp
            // This assures that in each loop cycle, the servo only moves by the 
            // MAX_DEG_PER_STEP constant and doesn't jerk.
            if (TURNTABLE_MAX_DEG_PER_STEP > 0) {
                double error = turntable_targetDeg - turntable_currentDeg;
                double step = Range.clip(error, -TURNTABLE_MAX_DEG_PER_STEP, TURNTABLE_MAX_DEG_PER_STEP);
                turntable_currentDeg += step;
            } else {
                turntable_currentDeg = turntable_targetDeg;
            }

            // Command the servos
            turntable.setPosition(degToPos(turntable_currentDeg, TURNTABLE_MAX_DEGREES));

            // Telemetry for both servos
            telemetry.addData("Turntable Target (deg)", "%.1f", turntable_targetDeg);
            telemetry.addData("Turntable Command (deg)", "%.1f", turntable_currentDeg);
            telemetry.addData("Turntable Position (0..1)", "%.3f", degToPos(turntable_currentDeg, TURNTABLE_MAX_DEGREES));
            telemetry.addData("Left Stick X", "%.3f", leftStickX);
            telemetry.addData("Right Stick Y", "%.3f", rightStickY);
            telemetry.update();

            sleep(20); // ~50 Hz loop
        }
    }

    private double degToPos(double deg, double maxDegrees) {
        // Map 0..200° -> 0.0..1.0
        double pos = deg / maxDegrees;
        return Range.clip(pos, 0.0, 1.0);
    }
}