/*
 * Copyright (c) 2025 Base 10 Assets, LLC
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
 * Neither the name of NAME nor the names of its contributors may be used to
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels," and two servos
 * which feed that launcher.
 *
 * This robot starts up against the goal and launches all three projectiles before driving away
 * off the starting line.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name="GoBildaStarterBotAutoMecanum", group="Production")
public class GoBildaStarterBotAutoMecanum extends OpMode
{

    final double FEED_TIME = 0.20; //The feeder servos run this long when a shot is requested.

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    /*
     * The number of seconds that we wait between each of our 3 shots from the launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes the likelihood
     * that each shot will score.
     */
    final double TIME_BETWEEN_SHOTS = 2;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either side of the
     * robot. Track width is used to determine the amount of linear distance each wheel needs to
     * travel to create a specified rotation of the robot.
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;

    int shotsToFire = 3; //The number of shots to fire in this auto.

    // === TUNE ME: straight-line speed constant (mm/sec) for a given power ===
    // Measure your robot's travel: drive at known power for N seconds, measure distance in mm, divide.
    public static double SPEED_MM_PER_SEC_AT_POWER_0p5 = 610; // at full power robot drives 1220mm in ~ 1 second

    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private LED led1LeftGreen = null;
    private LED led1LeftRed = null;
    private LED led1RightGreen = null;
    private LED led1RightRed = null;

    private LED led2LeftGreen = null;
    private LED led2LeftRed = null;
    private LED led2RightGreen = null;
    private LED led2RightRed = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private IMU imu = null;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "LaunchState." It reflects the current condition of the shooter motor when we request a shot.
     * It starts at IDLE. When a shot is requested from the user, it'll move into PREPARE then LAUNCH.
     * We can use higher level code to cycle through these states, but this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits."
     */
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }

    /*
     * Here we create the instance of LaunchState that we use in code. This creates a unique object
     * which can store the current condition of the shooter. In other applications, you may have
     * multiple copies of the same enum which have different names. Here we just have one.
     */
    private LaunchState launchState;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        DRIVE_BACKWARDS,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE;
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;

    // Create an enum to keep track whether or not the robot is moving. The three states
    // here are STOPPED when it's not moving. DRIVING when it's going forward, backwards or straffing
    // and ROTATING when it's rotating.

    private enum RobotInMotion {
        STOPPED,
        DRIVING,
        ROTATING;
    }

    private RobotInMotion robotInMotion = RobotInMotion.STOPPED;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;


        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
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
        imu = hardwareMap.get(IMU.class, "imu");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions.
         */
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Run the drive motors without encoders.
         */
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, and it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Here we set the aforementioned PID coefficients. You shouldn't have to do this for any
         * other motors on this robot.
         */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));

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


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        /*
         * We also set the servo power to 0 here to make sure that the servo controller is booted
         * up and ready to go.
         */
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);

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

        // LED 2 is used to indicate whether the robot's current yaw is
        // near zero. If it's not, the robot has been initialized before
        // it was placed on the field.

        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double currentAngle = ypr.getYaw(AngleUnit.DEGREES);

        if (currentAngle > -0.1 && currentAngle < 0.1) {
            led2RightGreen.on();
            led2LeftGreen.on();

            led2RightRed.off();
            led2LeftRed.off();
        } else {
            led2RightGreen.off();
            led2LeftGreen.off();

            led2RightRed.on();
            led2LeftRed.on();
        }

        telemetry.addData("Press SQUARE", "for BLUE");
        telemetry.addData("Press CIRCLE", "for RED");
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Current Angle Header", currentAngle);
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {

        // Turn off the LEDs
        led1RightGreen.off();
        led1LeftGreen.off();
        led1RightRed.off();
        led1LeftRed.off();
        led2RightGreen.off();
        led2LeftGreen.off();
        led2RightRed.off();
        led2LeftRed.off();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {

        // LED 1 is used to indicate whether or not the launcher
        // is at proper velocity.

        if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            led1RightRed.off();
            led1LeftRed.off();

            led1RightGreen.on();
            led1LeftGreen.on();
        } else {
            led1RightGreen.off();
            led1LeftGreen.off();

            led1RightRed.on();
            led1LeftRed.on();
        }

        // LED 2 is used to indicate whether the limelight is reporting that
        // the robot is in the correct spot to reliably shoot a goal. The correct
        // spot is dependent on the currently selected alliance and

        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState){
            /*
             * Since the first state of our auto is LAUNCH, this is the first "case" we encounter.
             * This case is very simple. We call our .launch() function with "true" in the parameter.
             * This "true" value informs our launch function that we'd like to start the process of
             * firing a shot. We will call this function with a "false" in the next case. This
             * "false" condition means that we are continuing to call the function every loop,
             * allowing it to cycle through and continue the process of launching the first ball.
             */
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                /*
                 * A technique we leverage frequently in this code are functions which return a
                 * boolean. We are using this function in two ways. This function actually moves the
                 * motors and servos in a way that launches the ball, but it also "talks back" to
                 * our main loop by returning either "true" or "false". We've written it so that
                 * after the shot we requested has been fired, the function will return "true" for
                 * one cycle. Once the launch function returns "true", we proceed in the code, removing
                 * one from the shotsToFire variable. If shots remain, we move back to the LAUNCH
                 * state on our state machine. Otherwise, we reset the encoders on our drive motors
                 * and move onto the next state.
                 */
                if(launch(false)) {
                    shotsToFire = shotsToFire - 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        launcher.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */
                if(startDriveDistance(-600, DRIVE_SPEED, false)) {
                    autonomousState = AutonomousState.ROTATING;
                };
                break;

            case ROTATING:
                double robotRotationAngle = 50;

                if (alliance == Alliance.BLUE) {
                    robotRotationAngle = robotRotationAngle * -1;
                }

                if(rotate(ROTATE_SPEED, robotRotationAngle)){
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:

                double distance = 700;

                if (alliance == Alliance.BLUE){
                    distance = distance * -1.0;
                }

                if(startDriveDistance(distance, DRIVE_SPEED, true)){
                    autonomousState = AutonomousState.DRIVE_BACKWARDS;
                }
                break;
            case DRIVE_BACKWARDS:
                if(startDriveDistance(-300, DRIVE_SPEED, false)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     * @param shotRequested "true" if the user would like to fire a new shot, and "false" if a shot
     *                      has already been requested and we need to continue to move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched, "false" otherwise.
     */
    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    /** Start driving for an estimated distance (mm) and return whether the drive is complete. */
    public boolean startDriveDistance(double distanceMM, double basePower, boolean straffing) {
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
            telemetry.update();
            return false;
        }
    }

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
     */
    public boolean rotate(double speed, double targetAngle) {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double currentAngle = ypr.getYaw(AngleUnit.DEGREES);
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
            telemetry.update();

            return false; // rotation has not yet completed.

        } else {
            // Robot has completed rotation.
            setMecanum(0,0,0);
            robotInMotion = RobotInMotion.STOPPED;
            return true;
        }
    }
}



