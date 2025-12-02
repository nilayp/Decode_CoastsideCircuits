/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Shooting Data Collection OpMode
 * 
 * This OpMode is dedicated to collecting shooting accuracy data from different
 * field positions to build a data-driven shooting system.
 * 
 * Workflow:
 * 1. Drive to position -> OPTIONS to start experiment
 * 2. Shoot 3 balls manually  
 * 3. Report results: DPAD UP (0), DPAD RIGHT (1), DPAD DOWN (2), DPAD LEFT (3)
 * 4. SHARE to save data to CSV
 * 5. Repeat for next position
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.io.FileWriter;

@TeleOp(name = "Shooting Data Collector", group = "Data Collection")
@Disabled
public class ShootingDataCollector extends OpMode {

    // Hardware
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx launcher;
    private SparkFunOTOS myOtos;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private IMU imu;
    private LED led1LeftGreen = null;
    private LED led1LeftRed = null;
    private LED led1RightGreen = null;
    private LED led1RightRed = null;

    // Alliance selection
    private enum Alliance {
        RED,
        BLUE
    }
    private Alliance alliance = Alliance.RED;

    // Shooting experiment system
    private enum ShootingState {
        IDLE,                // Ready to start new experiment
        WAITING_FOR_RESULT   // Shots taken, waiting for dpad input
    }

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    private ShootingState shootingState = ShootingState.IDLE;
    private SparkFunOTOS.Pose2D experimentStartPos;
    private double experimentStartYaw;
    private int shotsScored = 0;
    private int experimentNumber = 0;
    private FileWriter shootingLog = null;
    
    // Button state tracking
    private boolean lastOptionsButton = false;
    private boolean lastShareButton = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;

    // Drive settings
    private double drivePower = 0.5;

    // Launch settings
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1750;  // Value needed to reliably shoot from back launch zone
    final double LAUNCHER_MIN_VELOCITY = 1700;     // 50 tick tolerance for "ready to fire"
    ElapsedTime feederTimer = new ElapsedTime();
  
    // PIDF Tuning Variables - Adjust these for tuning
    double kP = 50.0;  // Proportional gain
    double kI = 0.0;    // Integral gain  
    double kD = 0.0;    // Derivative gain
    double kF = 14.166;   // Feedforward gain

    @Override
    public void init() {
         launchState = LaunchState.IDLE;

        // Declare our motors & other gear
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        imu = hardwareMap.get(IMU.class, "imu");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        led1LeftGreen = hardwareMap.get(LED.class, "led1_left_green");
        led1LeftRed = hardwareMap.get(LED.class, "led1_left_red");
        led1RightGreen = hardwareMap.get(LED.class, "led1_right_green");
        led1RightRed = hardwareMap.get(LED.class, "led1_right_red");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Initialize the IMU and set the correct orientation
         */

        IMU.Parameters parms = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )
        );
        imu.initialize(parms);
        imu.resetYaw();

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();
        
        // Initialize shooting experiment logging
        initializeShootingLog();



        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized - Ready for data collection");
    }

    @Override
    public void init_loop() {
        // Alliance selection
        if (gamepad1.circle) {
            alliance = Alliance.RED;
        } else if (gamepad1.square) {
            alliance = Alliance.BLUE;
        }

        // LED 1 is used to indicate whether the robot is positioned
        // for the red goal or the blue goal.
        // RED LED is for RED GOAL.
        // GREEN LED BLUE GOAL.

        if (alliance == Alliance.RED) {
            led1RightRed.on();
            led1LeftRed.on();

            led1RightGreen.off();
            led1LeftGreen.off();
        } else {
            led1RightGreen.on();
            led1LeftGreen.on();

            led1RightRed.off();
            led1LeftRed.off();
        }

        telemetry.addData("Press SQUARE", "for BLUE alliance");
        telemetry.addData("Press CIRCLE", "for RED alliance");
        telemetry.addData("Current Alliance", "%s", alliance);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Drive control
        handleDriving();

        handleShooting();

        // Handle shooting experiments
        handleShootingExperiment();

        // Display telemetry
        displayTelemetry();
    }

    private void handleDriving() {
        // Speed control
        if (gamepad1.left_bumper) {
            drivePower = (drivePower == 1.0) ? 0.5 : 1.0;
        }

        double forward = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;

        setMecanum(forward, turn, strafe, drivePower);
    }
    private void setMecanum(double forward, double turn, double strafe, double drivePower) {

        double frontLeftPower = (forward + turn + strafe) * drivePower;
        double frontRightPower = (forward - turn - strafe) * drivePower;
        double backLeftPower = (forward + turn - strafe) * drivePower;
        double backRightPower = (forward - turn + strafe) * drivePower;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    private void handleShooting() {
        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.triangle) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.circle) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
    /**
     * Handle shooting experiment state machine
     */
    private void handleShootingExperiment() {
        // OPTIONS button - start new experiment (only on button press, not hold)
        if (gamepad1.options && !lastOptionsButton && shootingState == ShootingState.IDLE) {
            startShootingExperiment();
        }
        lastOptionsButton = gamepad1.options;

        // DPAD buttons - record shot results (only when waiting for results)
        if (shootingState == ShootingState.WAITING_FOR_RESULT) {
            if (gamepad1.dpad_up && !lastDpadUp) {
                shotsScored = 0;  // 0 successful shots
            } else if (gamepad1.dpad_right && !lastDpadRight) {
                shotsScored = 1;  // 1 successful shot
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                shotsScored = 2;  // 2 successful shots
            } else if (gamepad1.dpad_left && !lastDpadLeft) {
                shotsScored = 3;  // 3 successful shots
            }
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;

        // SHARE button - save experiment data (only on button press, not hold)
        if (gamepad1.share && !lastShareButton && shootingState == ShootingState.WAITING_FOR_RESULT && shotsScored >= 0) {
            saveShootingExperiment();
        }
        lastShareButton = gamepad1.share;
    }

    /**
     * Start a new shooting experiment by recording current position
     */
    private void startShootingExperiment() {
        experimentStartPos = myOtos.getPosition();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        experimentStartYaw = orientation.getYaw(AngleUnit.DEGREES);

        experimentNumber++;
        shootingState = ShootingState.WAITING_FOR_RESULT;
        shotsScored = -1; // Invalid until DPAD is pressed
    }

    /**
     * Save shooting experiment data to CSV and reset for next experiment
     */
    private void saveShootingExperiment() {
        if (shootingLog == null) return;

        try {
            double successRate = (double) shotsScored / 3.0 * 100.0;

            // Write to CSV - just position (X,Y), yaw, and shooting results
            shootingLog.write(String.format(java.util.Locale.US, 
                "%d,%s,%.6f,%.6f,%.2f,%d,%.1f\n",
                experimentNumber, alliance, experimentStartPos.x, experimentStartPos.y,
                experimentStartYaw, shotsScored, successRate));
            shootingLog.flush();

            // Reset for next experiment
            shootingState = ShootingState.IDLE;
            shotsScored = -1;
            
            telemetry.addLine();
            telemetry.addLine("✅ EXPERIMENT SAVED!");
            telemetry.addData("Experiment #", "%d", experimentNumber);
            telemetry.addData("Results", "%d/3 shots (%.1f%%)", shotsScored, successRate);

        } catch (Exception e) {
            telemetry.addData("Save Error", e.getMessage());
        }
    }

    /**
     * Initialize shooting experiment logging system
     */
    private void initializeShootingLog() {
        try {
            String timestamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", java.util.Locale.US).format(new java.util.Date());
            String filename = "/sdcard/FIRST/shooting_experiments_" + timestamp + ".csv";

            shootingLog = new java.io.FileWriter(filename);
            shootingLog.write("Experiment,Alliance,X_m,Y_m,Yaw_deg,Shots_Scored,Success_Rate\n");
            shootingLog.flush();

            telemetry.addData("Shooting Log", "Initialized: " + filename);
        } catch (Exception e) {
            telemetry.addData("Log Error", e.getMessage());
        }
    }

    private void displayTelemetry() {
        // Current position
        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("Position", "X:%.3f Y:%.3f IMU Heading:%.3f", currentPos.x, currentPos.y, currentYaw);
        telemetry.addData("Drive Power", "%.1f%%", drivePower * 100);
        telemetry.addData("Launcher", "Current (%.2f), Target (%.2f), Min (%.2f)", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addLine();

        // Shooting experiment status
        telemetry.addLine("=== SHOOTING DATA COLLECTION ===");
        telemetry.addData("Alliance", "%s", alliance);
        telemetry.addData("Experiment #", "%d", experimentNumber);
        telemetry.addData("State", "%s", shootingState);
        telemetry.addLine();

        switch (shootingState) {
            case IDLE:
                telemetry.addData("Ready", "🎯 Drive to position, press OPTIONS to start");
                telemetry.addLine();
                telemetry.addData("Controls", "Left Stick: Drive | Right Stick: Rotate");
                telemetry.addData("", "Left Bumper: Toggle speed");
                break;
            case WAITING_FOR_RESULT:
                telemetry.addData("Position Recorded", "X:%.3f Y:%.3f Yaw:%.1f°", 
                    experimentStartPos.x, experimentStartPos.y, experimentStartYaw);
                telemetry.addData("Status", "🏀 SHOOT 3 BALLS, then report results:");
                telemetry.addLine();
                telemetry.addData("Results", "🎮 Use DPAD to report:");
                telemetry.addData("  DPAD UP", "0 shots scored");
                telemetry.addData("  DPAD RIGHT", "1 shot scored");
                telemetry.addData("  DPAD DOWN", "2 shots scored");
                telemetry.addData("  DPAD LEFT", "3 shots scored");
                
                if (shotsScored >= 0) {
                    telemetry.addLine();
                    telemetry.addData("Recorded", "✅ %d/3 shots (%.1f%%)", shotsScored, (shotsScored/3.0)*100);
                    telemetry.addData("Next", "Press SHARE to save data");
                }
                break;
        }

        telemetry.addLine();
        telemetry.addData("File", "shooting_experiments_*.csv in /sdcard/FIRST/");
        telemetry.update();
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        myOtos.setLinearUnit(DistanceUnit.METER);
        // myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // OTOS Mounting Position: 6cm LEFT of robot center
        // X=0 (no forward/back offset), Y=0.06 (6cm left is POSITIVE Y), H=0 (no rotation)
        // This tells OTOS where it's mounted relative to robot center
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.0, 0.06, 0.0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        // Set calibration scalar from constant (update OTOS_LINEAR_SCALAR after testing)
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);    // Angular not used (we use IMU for heading)

        // ENHANCED IMU CALIBRATION - Robot must be completely still!
        telemetry.addLine("OTOS IMU CALIBRATION - KEEP ROBOT PERFECTLY STILL!");
        telemetry.addLine("Calibrating for 5 seconds...");
        telemetry.update();

        // Extended calibration with more samples for drift reduction
        myOtos.calibrateImu(500, true);  // 500 samples (longer calibration)

        telemetry.addLine("Calibration complete. Resetting tracking...");
        telemetry.update();

        // Multiple reset cycles to ensure clean start
        for (int i = 0; i < 3; i++) {
            myOtos.resetTracking();
            try { Thread.sleep(200); } catch (InterruptedException e) {}
        }

        // Wait a moment for stabilization
        try { Thread.sleep(500); } catch (InterruptedException e) {}

        // Force position to exact zero
        SparkFunOTOS.Pose2D zeroPosition = new SparkFunOTOS.Pose2D(0.000, 0.000, 0.000);
        myOtos.setPosition(zeroPosition);

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        // Verify calibration worked
        SparkFunOTOS.Pose2D verifyPos = myOtos.getPosition();

        telemetry.addLine("=== OTOS CALIBRATION COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine(String.format("Hardware: v%d.%d | Firmware: v%d.%d", hwVersion.major, hwVersion.minor, fwVersion.major, fwVersion.minor));
        telemetry.addLine();
        telemetry.addLine("CONFIGURATION:");
        telemetry.addData("Mounting", "6cm LEFT of robot center");
        telemetry.addData("Offset", "X:0.00 Y:0.06 H:0.0°");
        telemetry.addData("IMU Calibration", "500 samples (extended)");
        telemetry.addData("Initial Position", "X:%.4f Y:%.4f H:%.2f°", verifyPos.x, verifyPos.y, Math.toDegrees(verifyPos.h));
        telemetry.addLine();
        telemetry.addLine("Monitor for drift while stationary!");
        telemetry.addData("Acceptable Drift", "< 0.01m position, < 0.1°/min heading");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Close shooting log file
        if (shootingLog != null) {
            try {
                shootingLog.close();
            } catch (Exception e) {
                // Ignore close errors
            }
        }
    }
}