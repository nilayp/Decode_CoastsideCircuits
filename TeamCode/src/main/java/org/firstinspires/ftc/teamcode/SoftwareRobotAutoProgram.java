/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This OpMode is used by the Software Robot to demonstrate the use an autonomous program for
 * our robot.
 */
@Autonomous(name = "SoftwareRobotAutoProgram", group = "Software")
@Disabled
public class SoftwareRobotAutoProgram extends OpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos = null;
    DcMotor leftDrive = null;
    DcMotor rightDrive = null;
    IMU imu = null;
    IMU.Parameters parms = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    )
    );
    double rightPower = 0.0;
    double leftPower = 0.0;
    double speed_factor = 0.50;
    /*
     * Here we capture a few variables used in driving the robot.
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;

    // TODO: let's determine how far we can travel in one second.
    double millimeters_per_second = 776.0;
    ElapsedTime driveTimer;

    // TODO: Set up the state machine. What are the different states that we need to be prepared for?
    private enum AutonomousState {
        MOVE_AWAY_FROM_WALL,
        ROTATE_AWAY_FROM_GOAL,
        MOVE_TOWARDS_RAMP,
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

    private Alliance alliance;

    private enum DrivingState {
        STOPPED,
        MOVING,
        ROTATING;
    }
    private DrivingState drivingState;

    @Override
    public void init() {
        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        // Get motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        // Get built-in IMU
        imu = hardwareMap.get(IMU.class, "imu");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        /*
         * Initialize the IMU and set the correct orientation
         */

        imu.initialize(parms);
        imu.resetYaw();

        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle from both the OTOS and built-in IMU sensors.
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Status", "Initialized");
        // Log the OTOS position to the telemetry
        telemetry.addData("OTOS X coordinate (m)", pos.x);
        telemetry.addData("OTOS Y coordinate (m)", pos.y);
        telemetry.addData("OTOS Heading angle (degrees)", pos.h);
        telemetry.addData("IMU Yaw", ypr.getYaw(AngleUnit.DEGREES));
        telemetry.update();

        alliance = Alliance.RED;
        autonomousState = AutonomousState.MOVE_AWAY_FROM_WALL;
}

    /*
     * This loop runs after the init is pushed.
     * up and ready to go.
     */
    @Override
    public void init_loop() {

        // In the init loop, we need to decide whether we are shooting for the
        // red or blue alliance.
        // TODO: Let's have square be for Blue and circle be for Red.
        // the default should be red. Setup telemetry so you can see it.
        // There should be instructions on the screen
        // telemetry.addLine("")
        if (gamepad1.square) {
            alliance = Alliance.BLUE;
        }
        if (gamepad1.circle) {
            alliance = Alliance.RED;
        }
        telemetry.addLine("Press gamepad1 square for BLUE.");
        telemetry.addLine("Press gamepad1 circle for RED.");
        telemetry.addData("Current Alliance is: ", alliance);
        telemetry.update();
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
    }

    /*
     * This code REPEATEDLY after the driver hits START.
     */
    @Override
    public void loop() {

        // TODO: Add a switch statement to work through the states of the state machine.
        // Write any helper functions that are required to for example, drive the robot,
        // rotate the robot, etc.

        switch (autonomousState) {
            case MOVE_AWAY_FROM_WALL:
                // move backwards 920mm
                if(driveByDistance(-920, 0.50)) {
                    autonomousState = AutonomousState.ROTATE_AWAY_FROM_GOAL;
                }
            case ROTATE_AWAY_FROM_GOAL:
                // 45 degrees
                double angle = 45.0;
                if (alliance == Alliance.BLUE) {
                    angle = angle * -1;
                }
                if(rotateByAngle(angle, 0.20)){
                    autonomousState = AutonomousState.MOVE_TOWARDS_RAMP;
                }
            case MOVE_TOWARDS_RAMP:
                // move forward 350mm
                if (driveByDistance(350, 0.50)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
        }

        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        // Reset the tracking if the user requests it
        if (gamepad1.triangle) {
            myOtos.resetTracking();
        }

        // Re-calibrate the OTOS IMU if the user requests it
        if (gamepad1.square) {
            myOtos.calibrateImu();
        }

        // Re-initialize the built-in IMU if the user requests it
        if (gamepad1.circle) {
            imu.initialize(parms);
        }

        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();

        // Inform user of available controls
        telemetry.addLine("Press Triangle on Gamepad to reset OTOS tracking");
        telemetry.addLine("Press Square on Gamepad to calibrate the OTOS IMU");
        telemetry.addLine("Press Circle to re-initialize built-in IMU");
        telemetry.addLine();

        // Log the position to the telemetry
        telemetry.addData("OTOS X coordinate (m)", pos.x);
        telemetry.addData("OTOS Y coordinate (m)", pos.y);
        telemetry.addData("OTOS Heading angle (degrees)", pos.h);
        telemetry.addData("IMU Yaw", ypr.getYaw(AngleUnit.DEGREES));

        // Update the telemetry on the driver station
        telemetry.update();
    }
    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double rotate, double speed_factor) {
        leftPower = (forward + rotate) * speed_factor;
        rightPower = (forward - rotate) * speed_factor;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
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
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
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
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

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

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
    private boolean driveByDistance(double millimeters_to_drive, double speed) {

        // The driveByDistance code has two parts - the initialization which should
        // only run once. And the code that runs in a loop each time it's called.
        // For initialization, set the drive state to active, then it computes the duration
        // of the drive by dividing the distance by the speed. Remember to use the absolute
        // value of the distance Math.abs() as you don't want a negative duration.
        // set the power to be positive if you are going forward and negative if you
        // are going backwards and set a timer (driveTimer.reset())
        // Then, outside of the init, you want to determine the elapsedDuration
        // timer.seconds(). If the elapsed duration >= drive duration, stop the motors
        // set the state to stop driving and return true.
        // Else - drive using the arcadeDrive function.
        // return false to the drive loop will run again.

        if (drivingState == DrivingState.STOPPED) {
            driveTimer.reset();
            drivingState = DrivingState.MOVING;
            return false;
        }
        double runTimeSeconds = Math.abs(millimeters_to_drive) / millimeters_per_second;
        double elapsedDuration = driveTimer.seconds();
        if (elapsedDuration >= runTimeSeconds) {
            arcadeDrive(0,0,0);
            drivingState = DrivingState.STOPPED;
            return true;
        } else {
            if (millimeters_to_drive < 0) {
                speed = speed * -1.0;
            }
            arcadeDrive(speed, 0, 1.0);
            return false;
        }
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
    private boolean rotateByAngle(double angle_to_rotate, double speed) {

        // The rotateByAngle code has two parts - the initialization which should
        // only run once. And the code that runs in a loop each time it's called.
        // you need to set the drive state to rotate.
        // Likely, there is nothing to initialize unless you want a timeout timer
        // get the currentAngle from the IMU
        // YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        // double currentAngle = ypr.getYaw(AngleUnit.DEGREES);
        // Calculate the error using the wrapAngleDeg function to ensure angles
        // stay between -180 to 180. This is the fastest way to rotate. For example, if
        // you need to rotate 270 degrees, having it rotate -90 would be faster.
        // double error = wrapAngDegree (targetAngle - currentAngle);
        // check whether we are 1 degree from the target. If so, stop the rotation and robot
        // if not, rotate at the speed. After the movement method is called, calculate the
        // error again so you can show it on the screen.
        // return false if rotating and true if rotation is complete.

        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        double currentAngle = ypr.getYaw(AngleUnit.DEGREES);
        double difference = wrapAngleDeg(angle_to_rotate - currentAngle);
        if (difference < 0) {
            speed = speed * -1.0;
        }
        arcadeDrive(0, speed,1.0);
               
        return true;
    }
}
