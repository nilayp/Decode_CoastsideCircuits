package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccLifter {

    private DcMotor leftLiftMotor = null;
    private DcMotor rightLiftMotor = null;

    private final ElapsedTime liftTimer = new ElapsedTime();

    // === TUNE ME: # of seconds to lift ===
    public static double MAX_LIFT_SECONDS = 24;

    final double DEFAULT_LIFT_SPEED = 0.2;
    double currentLiftSpeed = 0.0;

    private enum AutoLiftState {
        IDLE,
        INIT,
        GOING_UP,
        STOP
    }

    private AutoLiftState autoLiftState;

    public void init(HardwareMap map) {

        // Declare our motors & other gear
        // Make sure your ID's match your configuration

        leftLiftMotor = map.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = map.get(DcMotor.class, "rightLiftMotor");

        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure both directions are explicit so motors move together
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLiftMotor.setZeroPowerBehavior(BRAKE);
        rightLiftMotor.setZeroPowerBehavior(BRAKE);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentLiftSpeed = 0.0;
        autoLiftState = AutoLiftState.IDLE;
    }

    public void runTeleOpLoop(Gamepad gamepad, Telemetry telemetry) {

        // Up / Down set direction directly
        if (gamepad.dpad_up) {
            currentLiftSpeed = DEFAULT_LIFT_SPEED;
        } else if (gamepad.dpad_down) {
            currentLiftSpeed = -DEFAULT_LIFT_SPEED;
        }

        // dpad_right increases magnitude (preserve sign; if zero, start positive)
        if (gamepad.dpad_right) {
            double sign = Math.signum(currentLiftSpeed);
            if (sign == 0.0) sign = 1.0;
            currentLiftSpeed = Math.copySign(Math.min(1.0, Math.abs(currentLiftSpeed) + 0.10), sign);
        }

        // dpad_left decreases magnitude (preserve sign)
        if (gamepad.dpad_left) {
            double sign = Math.signum(currentLiftSpeed);
            if (sign != 0.0) {
                currentLiftSpeed = Math.copySign(Math.max(0.0, Math.abs(currentLiftSpeed) - 0.10), sign);
            }
        }

        // share button stops
        if (gamepad.share) {
            currentLiftSpeed = 0.0;
            autoLiftState = AutoLiftState.STOP;
        }

        // the big button has it go up for 25 seconds
        if (gamepad.touchpad) {
            telemetry.addLine("touchpad pushed");
            lift(true);
        }

        lift(false);

        // apply to motors if initialized
        if (leftLiftMotor != null && rightLiftMotor != null) {
            leftLiftMotor.setPower(currentLiftSpeed);
            rightLiftMotor.setPower(currentLiftSpeed);
        } else {
            telemetry.addData("Lift", "motors not initialized");
        }

        telemetry.addData("Lift Power", currentLiftSpeed);
        telemetry.addData("Auto Lift State", autoLiftState);
        telemetry.addData("Lift Timer", liftTimer.seconds());
        telemetry.addData("dpad_up", gamepad.dpad_up);
        telemetry.addData("dpad_down", gamepad.dpad_down);
        telemetry.addData("dpad_right", gamepad.dpad_right);
        telemetry.addData("dpad_left", gamepad.dpad_left);
    }

    private void lift(boolean liftRequested) {
        switch (autoLiftState) {
            case IDLE:
                if (liftRequested) {
                    autoLiftState = AutoLiftState.INIT;
                }
                break;
            case INIT:
                liftTimer.reset();
                currentLiftSpeed = 1.0;
                autoLiftState = AutoLiftState.GOING_UP;
                break;
            case GOING_UP:
                if (liftTimer.seconds() > MAX_LIFT_SECONDS) {
                    autoLiftState = AutoLiftState.STOP;
                }
                break;
            case STOP:
                currentLiftSpeed = 0.0;
                autoLiftState = AutoLiftState.IDLE;
                break;
        }
    }
}
