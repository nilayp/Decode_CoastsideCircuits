package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccDrive {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    // // Constants and other variables used for the AUTONOMOUS program

    private final ElapsedTime driveTimer = new ElapsedTime();

    // === TUNE ME: straight-line speed constant (mm/sec) for a given power ===
    // Measure your robot's travel: drive at known power for N seconds, measure distance in mm, divide.
    public static double SPEED_MM_PER_SEC_AT_POWER_0p5 = 610; // at full power robot drives 1220mm in ~ 1 second

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either side of the
     * robot. Track width is used to determine the amount of linear distance each wheel needs to
     * travel to create a specified rotation of the robot.
     */
    final double AUTO_DRIVE_SPEED = 0.5;
    final double AUTO_ROTATE_SPEED = 0.3;

    // Create an enum to keep track whether or not the robot is moving. The three states
    // here are STOPPED when it's not moving. DRIVING when it's going forward, backwards or straffing
    // and ROTATING when it's rotating.

    private enum RobotInMotion {
        STOPPED,
        DRIVING,
        ROTATING;
    }

    private RobotInMotion robotInMotion = RobotInMotion.STOPPED;

    public double teleOpDrivePower = 1.0;

    public void init(HardwareMap map) {

        // Declare our motors & other gear
        // Make sure your ID's match your configuration

        frontLeftMotor = map.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = map.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = map.get(DcMotor.class, "frontRightMotor");
        backRightMotor = map.get(DcMotor.class, "backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
    }

    public void runTeleOpLoop(Gamepad gamepad, Telemetry telemetry) {

        if (gamepad.left_bumper) {
            if (teleOpDrivePower == 1.00) {
                teleOpDrivePower = 0.5;
            } else {
                teleOpDrivePower = 1.00;
            }
        }

        // Manual control only
        double forward = gamepad.left_stick_y * teleOpDrivePower;
        double turn = -gamepad.right_stick_x * teleOpDrivePower;
        double strafe = -gamepad.left_stick_x * teleOpDrivePower;

        setMecanum(forward, turn, strafe);

        telemetry.addData("Drive Power", teleOpDrivePower);
    }

    public void runTeleOpSensorsLoop(Gamepad gamepad, Telemetry telemetry, ccIMU ccimu, ccLimelight ccLimelight, ccLED leftLED, ccLED rightLED) {

        if (gamepad.left_bumper) {
            if (teleOpDrivePower == 1.00) {
                teleOpDrivePower = 0.5;
            } else {
                teleOpDrivePower = 1.00;
            }
        }

        // Manual control only
        double forward = gamepad.left_stick_y * teleOpDrivePower;
        double turn = -gamepad.right_stick_x * teleOpDrivePower;
        double strafe = -gamepad.left_stick_x * teleOpDrivePower;

        double currentTx = ccLimelight.getTx(ccimu);
        if (abs(currentTx) < 2.0) {
            leftLED.setGreenLed();
            rightLED.setGreenLed();
        } else {
            leftLED.setRedLed();
            rightLED.setRedLed();
        }

        if (gamepad.cross) {
            if (currentTx != -360.0 && abs(currentTx) > 0) {
                turn = rotateToTx(currentTx);
            }
        }

        setMecanum(forward, turn, strafe);

        telemetry.addData("Drive Power", teleOpDrivePower);
    }

    public void setMecanum(double forward, double turn, double strafe) {

        double frontLeftPower = (forward + turn + strafe);
        double frontRightPower = (forward - turn - strafe);
        double backLeftPower = (forward + turn - strafe);
        double backRightPower = (forward - turn + strafe);

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Start driving for an estimated distance (mm) and return whether the drive is complete.
     */
    public boolean startDriveDistance(double distanceMM, boolean straffing, Telemetry telemetry) {
        double basePower = AUTO_DRIVE_SPEED;
        if (robotInMotion == RobotInMotion.STOPPED) {
            robotInMotion = RobotInMotion.DRIVING;
            driveTimer.reset();
        }

        double driveDurationSec = Math.abs(distanceMM) / SPEED_MM_PER_SEC_AT_POWER_0p5;

        if (distanceMM >= 0) {
            basePower = -basePower;
        }

        double driveElapsedDuration = driveTimer.seconds();
        if (driveElapsedDuration >= driveDurationSec) {
            // Drive is complete. Stop the robot and return true.

            setMecanum(0, 0, 0);
            robotInMotion = RobotInMotion.STOPPED;
            return true;
        } else {
            // Drive is still in progress.

            double forward = 0.0;
            double strafe = 0.0;

            if (straffing) {
                strafe = basePower;
            } else {
                forward = basePower;
            }

            setMecanum(forward, 0, strafe);

            telemetry.addData("Driving", "%.1f/%.1f sec", driveElapsedDuration, driveDurationSec);
            return false;
        }
    }

    private double wrapAngleDeg(double a) {
        while (a > 180) {
            a = a - 360;
        }
        while (a <= -180) {
            a = a + 360;
        }
        return a;
    }

    /**
     * Rotates the robot to a specific angle using the IMU.
     *
     * @param targetAngle The target angle to rotate to, in degrees.
     * @return True if the rotation is complete, false otherwise.
     */
    public boolean rotate(double targetAngle, ccIMU ccimu, Telemetry telemetry) {
        double speed = AUTO_ROTATE_SPEED;
        double currentAngle = ccimu.getYaw();
        double error = wrapAngleDeg(targetAngle - currentAngle);

        if (robotInMotion == RobotInMotion.STOPPED) {
            robotInMotion = RobotInMotion.ROTATING;
        }

        if (Math.abs(error) > 1) {
            if (error < 0) {
                speed = speed * -1.0;
            }

            setMecanum(0, speed, 0);
            error = wrapAngleDeg(targetAngle - currentAngle);

            telemetry.addData("Rotating", "Target: %.1f, Current: %.1f, Error: %.1f", targetAngle, currentAngle, error);
            return false; // rotation has not yet completed.

        } else {
            // Robot has completed rotation.
            setMecanum(0, 0, 0);
            robotInMotion = RobotInMotion.STOPPED;
            return true;
        }
    }

    /**
     * Rotates the robot to a Tx value 0 (+/-) as seen by the Limelight.
     *
     * @param currentTx
     * @return double value to rotate to
     */
    private double rotateToTx(double currentTx) {

        currentTx = currentTx / 25.0; // factor determined by trial and error.

        if (currentTx > 1.0) {
            currentTx = 1.0;
        } else if (currentTx < -1.0) {
            currentTx = -1.0;
        }
        return -currentTx;
    }
}
