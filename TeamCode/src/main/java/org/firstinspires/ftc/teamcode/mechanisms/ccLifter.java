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
    public static double MAX_LIFT_LEFT_SECONDS = 8.25;
    public static double MAX_LIFT_RIGHT_SECONDS = 8.50;

    final double DEFAULT_LIFT_SPEED = 0.2;
    double currentLeftLiftSpeed = 0.0;
    double currentRightLiftSpeed = 0.0;

    private enum AutoLiftState {
        IDLE,
        INIT,
        GOING_UP_BOTH,
        GOING_UP_RIGHT_MORE,
        STOP
    }

    private AutoLiftState autoLiftState;

    public void init(HardwareMap map) {

        // Declare our motors & other gear
        // Make sure your ID's match your configuration

        leftLiftMotor = map.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = map.get(DcMotor.class, "rightLiftMotor");

        // Ensure both directions are explicit so motors move together
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLiftMotor.setZeroPowerBehavior(BRAKE);
        rightLiftMotor.setZeroPowerBehavior(BRAKE);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentRightLiftSpeed = 0.0;
        currentLeftLiftSpeed = 0.0;
        autoLiftState = AutoLiftState.IDLE;
    }

    public void runTeleOpLoop(Gamepad gamepad, Telemetry telemetry) {

        // Up / Down set direction directly
        if (gamepad.dpad_up) {
            currentRightLiftSpeed = DEFAULT_LIFT_SPEED;
            currentLeftLiftSpeed = DEFAULT_LIFT_SPEED;
        } else if (gamepad.dpad_down) {
            currentRightLiftSpeed = -DEFAULT_LIFT_SPEED;
            currentLeftLiftSpeed = -DEFAULT_LIFT_SPEED;
        }

        // dpad_right increases magnitude (preserve sign; if zero, start positive)
        if (gamepad.dpad_right) {
            double signR = Math.signum(currentRightLiftSpeed);
            if (signR == 0.0) signR = 1.0;
            currentRightLiftSpeed = Math.copySign(Math.min(1.0, Math.abs(currentRightLiftSpeed) + 0.10), signR);
            double signL = Math.signum(currentLeftLiftSpeed);
            if (signL == 0.0) signL = 1.0;
            currentLeftLiftSpeed = Math.copySign(Math.min(1.0, Math.abs(currentLeftLiftSpeed) + 0.10), signL);
        }

        // dpad_left decreases magnitude (preserve sign)
        if (gamepad.dpad_left) {
            double signR = Math.signum(currentRightLiftSpeed);
            if (signR != 0.0) {
                currentRightLiftSpeed = Math.copySign(Math.max(0.0, Math.abs(currentRightLiftSpeed) - 0.10), signR);
            }
            double signL = Math.signum(currentLeftLiftSpeed);
            if (signL != 0.0) {
                currentLeftLiftSpeed = Math.copySign(Math.max(0.0, Math.abs(currentLeftLiftSpeed) - 0.10), signL);
            }
        }

        // share button stops
        if (gamepad.share) {
            currentRightLiftSpeed = 0.0;
            currentLeftLiftSpeed = 0.0;
            autoLiftState = AutoLiftState.STOP;
        }

        // the big button has it go up for 25 seconds
        if (gamepad.touchpad) {
            lift(true);
        }

        lift(false);

        // apply to motors if initialized
        if (leftLiftMotor != null && rightLiftMotor != null) {
            leftLiftMotor.setPower(currentLeftLiftSpeed);
            rightLiftMotor.setPower(currentRightLiftSpeed);
        } else {
            telemetry.addData("Lift", "motors not initialized");
        }

        telemetry.addData("Left Lift Power", currentLeftLiftSpeed);
        telemetry.addData("Right Lift Power", currentRightLiftSpeed);
        if (autoLiftState == AutoLiftState.GOING_UP_BOTH) {
            telemetry.addData("Auto Lift State", autoLiftState);
            telemetry.addData("Lift Timer", liftTimer.seconds());
        }
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
                currentLeftLiftSpeed = 1.0;
                currentRightLiftSpeed = 1.0;
                autoLiftState = AutoLiftState.GOING_UP_BOTH;
                break;
            case GOING_UP_BOTH:
                if (liftTimer.seconds() > MAX_LIFT_LEFT_SECONDS) {
                    autoLiftState = AutoLiftState.GOING_UP_RIGHT_MORE;
                }
                break;
            case GOING_UP_RIGHT_MORE:
                currentLeftLiftSpeed = 0.0;
                currentRightLiftSpeed = 1.0;
                if (liftTimer.seconds() > MAX_LIFT_RIGHT_SECONDS) {
                    autoLiftState = AutoLiftState.STOP;
                }
                break;
            case STOP:
                currentLeftLiftSpeed = 0.0;
                currentRightLiftSpeed = 0.0;
                autoLiftState = AutoLiftState.IDLE;
                break;
        }
    }
}
