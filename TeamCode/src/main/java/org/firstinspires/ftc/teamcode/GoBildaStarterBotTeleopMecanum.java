/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "GoBildaStarterBotTeleopMecanum", group = "Production")
public class GoBildaStarterBotTeleopMecanum extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    double drivePower = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     * 
     * Motor: 6000 RPM rated, achieving 2500 ticks/sec = 5450 RPM (91% of rated!)
     * Conversion: 2.18 RPM per tick/sec (confirmed accurate)
     * Target optimized for consistent high-performance launcher shots
     */
    final double LAUNCHER_TARGET_VELOCITY = 1750;  // Value needed to reliably shoot from back launch zone
    final double LAUNCHER_MIN_VELOCITY = 1700;     // 50 tick tolerance for "ready to fire"
    
    /*
    // Data-driven shooting rectangle (to be determined from field experiments)
    // TODO: Update these values based on shooting test data
    private static class ShootingRectangle {
        double minX, maxX, minY, maxY;  // Rectangle boundaries in meters
        
        ShootingRectangle(double minX, double maxX, double minY, double maxY) {
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;
        }
        
        boolean contains(double x, double y) {
            return x >= minX && x <= maxX && y >= minY && y <= maxY;
        }
    }
    
    // Shooting rectangles for each alliance (placeholder values - update from test data)
    private final ShootingRectangle RED_SHOOTING_RECT = new ShootingRectangle(-1.5, -0.5, 0.5, 1.5);
    private final ShootingRectangle BLUE_SHOOTING_RECT = new ShootingRectangle(0.5, 1.5, 0.5, 1.5);
    
    // Function to calculate optimal shooting yaw based on position
    // TODO: Replace with polynomial or lookup table from experimental data
    private double calculateOptimalYaw(double x, double y) {
        // Placeholder calculation - replace with data-driven approach
        if (alliance == Alliance.RED) {
            // For red alliance, approximate calculation
            // TODO: Use experimental data to create accurate function
            return Math.toDegrees(Math.atan2(3.7 - y, 0.0 - x));  // Aim toward red basket
        } else {
            // For blue alliance
            // TODO: Use experimental data to create accurate function  
            return Math.toDegrees(Math.atan2(3.7 - y, 0.0 - x));  // Aim toward blue basket
        }
    }
    final double YAW_TOLERANCE = 2.0;         // Yaw tolerance in degrees
    */
    // PIDF Tuning Variables - Adjust these for tuning
    double kP = 50.0;  // Proportional gain
    double kI = 0.0;    // Integral gain  
    double kD = 0.0;    // Derivative gain
    double kF = 14.166;   // Feedforward gain

    // Declare OpMode members.

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private LED led1LeftGreen = null;
    private LED led1LeftRed = null;
    private LED led1RightGreen = null;
    private LED led1RightRed = null;

    private LED led2LeftGreen = null;
    private LED led2LeftRed = null;
    private LED led2RightGreen = null;
    private LED led2RightRed = null;

    // private Limelight3A limelight;
    // private IMU imu;
    // private SparkFunOTOS myOtos = null;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    private enum Alliance {
        RED,
        BLUE;
    }

    private Alliance alliance = Alliance.RED;

    // Shooting rectangle alignment status
    /*
    private boolean inShootingRectangle = false;
    private boolean headingAligned = false;
    private double optimalYaw = 0.0;
    private boolean alignmentRequested = false;


    private enum RobotInAutomaticMotion {
        NO,
        DRIVING,
        ROTATING;
    }

    private RobotInAutomaticMotion robotInMotion = RobotInAutomaticMotion.NO;
*/
    /*
     * Code to run ONCE when the driver hits INIT
     */
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
        led1LeftGreen = hardwareMap.get(LED.class, "led1_left_green");
        led1LeftRed = hardwareMap.get(LED.class, "led1_left_red");
        led1RightGreen = hardwareMap.get(LED.class, "led1_right_green");
        led1RightRed = hardwareMap.get(LED.class, "led1_right_red");
        led2LeftGreen = hardwareMap.get(LED.class, "led2_left_green");
        led2LeftRed = hardwareMap.get(LED.class, "led2_left_red");
        led2RightGreen = hardwareMap.get(LED.class, "led2_right_green");
        led2RightRed = hardwareMap.get(LED.class, "led2_right_red");
        // imu = hardwareMap.get(IMU.class, "imu");
        // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

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


        IMU.Parameters parms = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )
        );
        imu.initialize(parms);
        imu.resetYaw();

        /*
         * Start the limelight so pipeline switching works properly

        limelight.start();

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
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


        telemetry.addData("Press SQUARE", "for BLUE");
        telemetry.addData("Press CIRCLE", "for RED");
        telemetry.addData("Selected Alliance", alliance);
        // telemetry.addData("Pipeline", alliance == Alliance.RED ? "0 (Red AprilTag #24)" : "1 (Blue AprilTag #20)");

        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle from both the OTOS and built-in IMU sensors.

        /*SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();

        telemetry.addLine("Sensor Readings - should be close to 0 for each one.");
        // Log the OTOS & IMU position to the telemetry
        telemetry.addData("OTOS X coordinate (m)", pos.x);
        telemetry.addData("OTOS Y coordinate (m)", pos.y);
        telemetry.addData("OTOS Heading angle (degrees)", pos.h);
        telemetry.addData("IMU Yaw", ypr.getYaw(AngleUnit.DEGREES));
         */
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        led1RightRed.off();
        led1LeftRed.off();
        led1RightGreen.off();
        led1LeftGreen.off();
        led2RightRed.off();
        led2LeftRed.off();
        led2RightGreen.off();
        led2LeftGreen.off();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            if (drivePower == 1.00) {
                drivePower = 0.5;
            } else {
                drivePower = 1.00;
            }
        }

        // Check shooting rectangle status and LED control
        // checkShootingStatus();
        // updateShootingLEDs();
        
        // Manual control only
        double forward = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double strafe = -gamepad1.left_stick_x;

        forward = forward * drivePower;
        turn = turn * drivePower;
        strafe = strafe * drivePower;

        setMecanum(forward, turn, strafe);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.triangle) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.circle) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
            led1RightGreen.on();
            led1LeftGreen.on();
            led1RightRed.off();
            led1LeftRed.off();

        } else {
            led1RightGreen.off();
            led1LeftGreen.off();
            led1RightRed.on();
            led1LeftRed.on();
        }
        /*
         * Alignment button - when pressed in shooting rectangle, align to optimal yaw

        if (gamepad1.cross && inShootingRectangle) {
            alignmentRequested = true;
            rotate(0.3, optimalYaw);
        }
        
        // Reset alignment request if not in rectangle
        if (!inShootingRectangle) {
            alignmentRequested = false;
        }
        */

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Drive Power", drivePower);
        telemetry.addData("Launcher", "Target (%.2f), Min (%.2f)", LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Launcher motorSpeed", launcher.getVelocity());

        /*
        // Simple two-sensor system status
        SparkFunOTOS.Pose2D currentOTOS = myOtos.getPosition();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
        
        telemetry.addLine();
        telemetry.addLine("=== SHOOTING SYSTEM STATUS ===");
        telemetry.addData("Position", "X:%.3f Y:%.3f meters", currentOTOS.x, currentOTOS.y);
        telemetry.addData("Heading", "%.1f°", currentYaw);
        
        // Distance calibration help
        double totalDistance = Math.sqrt(currentOTOS.x * currentOTOS.x + currentOTOS.y * currentOTOS.y);
        telemetry.addLine();

        
        // Shooting rectangle status
        telemetry.addLine();
        ShootingRectangle currentRect = (alliance == Alliance.RED) ? RED_SHOOTING_RECT : BLUE_SHOOTING_RECT;
        telemetry.addData("Alliance", "%s", alliance);
        telemetry.addData("Shooting Rectangle", "X:[%.1f,%.1f] Y:[%.1f,%.1f]", 
                         currentRect.minX, currentRect.maxX, currentRect.minY, currentRect.maxY);
        telemetry.addData("In Rectangle", "%s %s", inShootingRectangle ? "YES" : "NO", 
                         inShootingRectangle ? "✓" : "✗");
        
        if (inShootingRectangle) {
            telemetry.addData("Optimal Yaw", "%.1f°", optimalYaw);
            if (alignmentRequested) {
                double yawError = optimalYaw - currentYaw;
                while (yawError > 180) yawError -= 360;
                while (yawError < -180) yawError += 360;
                telemetry.addData("Yaw Error", "%.1f° (Target: ±%.1f°)", yawError, YAW_TOLERANCE);
                telemetry.addData("Heading Aligned", "%s %s", headingAligned ? "YES" : "NO", 
                                 headingAligned ? "✓" : "✗");
            } else {
                telemetry.addData("Alignment", "Press X to align heading");
            }
        }

        // LED status
        telemetry.addData("LED2 Left (Position)", inShootingRectangle ? "GREEN" : "RED");
        telemetry.addData("LED2 Right (Heading)", headingAligned ? "GREEN" : "RED");
        telemetry.addData("Ready to Fire", (inShootingRectangle && headingAligned) ? "YES - BOTH GREEN!" : "NO");
        */

        telemetry.update();

    }
    
    /**
     * Controls LEDs to indicate shooting readiness
     * Left LED2: Green when in shooting rectangle
     * Right LED2: Green when heading aligned (after alignment requested)

    private void updateShootingLEDs() {
        // Left LED2: Position status (green when in shooting rectangle)
        if (inShootingRectangle) {
            led2LeftGreen.on();
            led2LeftRed.off();
        } else {
            led2LeftGreen.off();
            led2LeftRed.on();
        }
        
        // Right LED2: Heading alignment status (green when aligned)
        if (headingAligned) {
            led2RightGreen.on();
            led2RightRed.off();
        } else {
            led2RightGreen.off();
            led2RightRed.on();
        }
    }
     */

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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
     * Checks if robot is in shooting rectangle and calculates optimal alignment

    private void checkShootingStatus() {
        // Get current position and heading
        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();
        double currentX = currentPos.x;
        double currentY = currentPos.y;
        
        // Get IMU heading
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
        
        // Check if in shooting rectangle
        ShootingRectangle currentRect = (alliance == Alliance.RED) ? RED_SHOOTING_RECT : BLUE_SHOOTING_RECT;
        inShootingRectangle = currentRect.contains(currentX, currentY);
        
        if (inShootingRectangle) {
            // Calculate optimal yaw for current position
            optimalYaw = calculateOptimalYaw(currentX, currentY);
            
            // Check heading alignment (only if alignment was requested)
            if (alignmentRequested) {
                double yawError = optimalYaw - currentYaw;
                // Normalize yaw error to [-180, 180]
                while (yawError > 180) yawError -= 360;
                while (yawError < -180) yawError += 360;
                
                headingAligned = Math.abs(yawError) < YAW_TOLERANCE;
            } else {
                headingAligned = false;
            }
        } else {
            headingAligned = false;
            alignmentRequested = false;
        }
    }
     */
    private void setMecanum(double forward, double turn, double strafe) {

        double frontLeftPower = (forward + turn + strafe);
        double frontRightPower = (forward - turn - strafe);
        double backLeftPower = (forward + turn - strafe);
        double backRightPower = (forward - turn + strafe);

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
/*
    private static double wrapAngleDeg(double a) {
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
     * @param speed The speed at which to rotate (0 to 1).
     * @param targetAngle The target angle to rotate to, in degrees.
     * @return True if the rotation is complete, false otherwise.

    public boolean rotate(double speed, double targetAngle) {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double currentAngle = ypr.getYaw(AngleUnit.DEGREES);
        double error = wrapAngleDeg(targetAngle - currentAngle);

        if (robotInMotion == RobotInAutomaticMotion.NO) {
            robotInMotion = RobotInAutomaticMotion.ROTATING;
        }

        if (Math.abs(error) > 1) {
            if (error < 0) {
                speed = speed * -1.0;
            }

            setMecanum(0, speed, 0);
            error = wrapAngleDeg(targetAngle - currentAngle);

            telemetry.addData("Rotating", "Target: %.1f, Current: %.1f, Error: %.1f", targetAngle, currentAngle, error);
            telemetry.update();

            return false; // rotation has not yet completed.

        } else {
            // Robot has completed rotation.
            setMecanum(0,0,0);
            robotInMotion = RobotInAutomaticMotion.NO;
            return true;
        }
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

    } */
}