/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@TeleOp(name = "SimpleSpinMotor", group = "Software")
@Disabled
public class SimpleSpinMotor extends LinearOpMode {
    DcMotor flywheel = null;

    double flywheel_speed = 0.50;
    double speed_factor = 0.50;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get motor
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setZeroPowerBehavior(BRAKE);

        telemetry.addData("Status", "Initialized");

        // Wait for the start button to be pressed
        waitForStart();

        // Loop until the OpMode ends
        while (opModeIsActive()) {
            // Reset the tracking if the user requests it
            if (gamepad1.left_bumper) {
                if (speed_factor == 0.50) {
                    speed_factor = 1.0;
                } else {
                    speed_factor = 0.50;
                }
            }

            flywheel_speed = gamepad1.left_stick_y;
            flywheel.setPower(flywheel_speed * speed_factor);

            // Log the motor's speed
            telemetry.addData("flywheel speed", flywheel_speed * speed_factor);

            // Update the telemetry on the driver station
            telemetry.update();
        }
    }
}