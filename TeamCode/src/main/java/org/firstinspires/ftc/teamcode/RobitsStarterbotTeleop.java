/* Copyright (c) <2025> <AndyMark>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of AndyMark nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This program provides a base-level code for the 2025-2026 Robits Starterbot for the DECODE challenge.
 * By default, this code controls the driving and launching of the robot via one gamepad
 * The drive style is selectable, but defaults to split arcade
 * Split Arcade: 
 * Driving Forward/Backward Left Joystick Forward/Backward 
 * Turning Left/Right       Right Joystick Left/Right
 *
 * Arcade: 
 * Driving Forward/Backward Left Joystick Forward/Backward 
 * Turning Left/Right       Left Joystick Left/Right
 *
 * Tank:
 * Left Side Drive          Left Joystick Forward/Backward 
 * Right Side Drive         Right Joystick Forward/Backward 
 *
 * The launch controls are by default are
 * Wind: Catapult down      Left Trigger
 * 
 * Release: Catapult up     Right Trigger
 * 
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RobitsStarterbotTeleop", group="Starterbot")
@Disabled

public class RobitsStarterbotTeleop extends LinearOpMode {

       // Declare OpMode members.
       private ElapsedTime runtime = new ElapsedTime();
       private DcMotor leftDrive;
       private DcMotor rightDrive;
       private DcMotor motorCatapult;
       
       double leftPower;
       double rightPower;

       //The following is a selection for the drive control style.
       private enum DriveStyle {
           SPLIT_ARCADE,
           ARCADE,
           TANK
       }
       private DriveStyle drive = DriveStyle.SPLIT_ARCADE;//Select drive mode

       /*
       * These constants control the catapult's wind and release speeds.
       * Adjust as needed.
       */
        private static final double WIND = 1.0;
        private static final double RELEASE = -1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app).
        */
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        motorCatapult = hardwareMap.dcMotor.get("catapult");

        /* Most robots need the motor on one side to be reversed to drive forward
        * If the robot is driving opposite directions than expected change
        * "REVERSE" to "FORWARD" or vice-versa. This also applies to the catapult.
        */ 
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        motorCatapult.setDirection(DcMotorSimple.Direction.FORWARD);    

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            //Drive Control
            double y = gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            
            /* Denominator is the largest motor power (absolute value) or 1
            * This ensures all the powers maintain the same ratio, but only when
            * at least one is out of the range [-1, 1]
            */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            
            
            if (drive == DriveStyle.SPLIT_ARCADE) { // split arcade drive
                leftPower = (y + rx) / denominator;
                rightPower = (y - rx) / denominator;
            } else if (drive == DriveStyle.ARCADE) { // arcade drive
                leftPower = (y + x) / denominator;
                rightPower = (y - x) / denominator;
            } else if (drive == DriveStyle.TANK) { // tank drive            
                leftPower = gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
            }
            
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            

            //Launch control
            if (gamepad1.left_trigger > 0) {
                motorCatapult.setPower(WIND);
            } else if (gamepad1.right_trigger > 0) {
                motorCatapult.setPower(RELEASE);
            } else{
                motorCatapult.setPower(0); //This turns the catapult motor off when not in use
            }            

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            
            // Telemetry data for drive type
            if (drive == DriveStyle.SPLIT_ARCADE) {
                telemetry.addData("Style","SplitArcade");   
            } else if (drive == DriveStyle.ARCADE) {
                telemetry.addData("Style","Arcade");   
            } else if (drive == DriveStyle.TANK) {
                telemetry.addData("Style","Tank");   
            }
            
            // Telemetry data for catapult status
            if (motorCatapult.getPower() == WIND) {
                telemetry.addData("Catapult Status","Wind");
            } else if (motorCatapult.getPower() == RELEASE) {
                telemetry.addData("Catapult Status","Release");
            } else {
                telemetry.addData("Catapult Status","Off");
            }
            
            telemetry.update();
        }
    }
}
