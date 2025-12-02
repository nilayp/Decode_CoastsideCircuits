package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ccOtos {
    private SparkFunOTOS myOtos = null;

    public void init(HardwareMap map, Telemetry telemetry) {
        myOtos = map.get(SparkFunOTOS.class, "sensor_otos");
        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos(telemetry);
    }

    public void loop(Telemetry telemetry) {

        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle from both the OTOS and built-in IMU sensors.

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        telemetry.addData("OTOS X coordinate (m)", pos.x);
        telemetry.addData("OTOS Y coordinate (m)", pos.y);
        telemetry.addData("OTOS Heading angle (degrees)", pos.h);
    }

    private void configureOtos(Telemetry telemetry) {
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
    }
}
