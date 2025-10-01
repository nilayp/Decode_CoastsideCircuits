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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This program provides a base-level code for the 2025-2026 Robits Starterbot for the DECODE challenge.
 * In the program, the robot will start centered against a goal, back up into launch range,
 * launch its gamepieces, and drive off the line. 
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "RobitsStarterbotAuto", group = "Starterbot")
@Disabled

public class RobitsStarterbotAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motorCatapult;

    private enum FieldSide {
        BLUE,
        RED
    }

    private FieldSide SIDE = FieldSide.BLUE; // Defaults to Blue

    private static final double WIND = 1.0;
    private static final double RELEASE = -1.0;

    static final double COUNTS_PER_MOTOR_REV = 537.6; // eg: NeveRest 19.2:1
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No External Gearing
    static final double WHEEL_DIAMETER_INCHES = 3.0;  // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        motorCatapult = hardwareMap.dcMotor.get("catapult");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCatapult.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* The below portion allows the user to select which side
        * of the field they are starting on.
        */
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) {
                SIDE = FieldSide.RED;
            } else if (gamepad1.x) {
                SIDE = FieldSide.BLUE;
            }

            telemetry.addData("Press X/SQUARE", "for BLUE");
            telemetry.addData("Press B/CIRCLE", "for RED");
            telemetry.addData("Selected Side", SIDE);
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            encoderDrive(DRIVE_SPEED, -12, -12, 5.0);   //Drive Backward
            launch();
            launch();
            launch();
            if (SIDE == FieldSide.BLUE) {
            encoderDrive(TURN_SPEED, 11, -11, 5.0); // Turn Right
            } else {
                encoderDrive(TURN_SPEED, -11, 11, 5.0); // Turn Left
            }
            encoderDrive(DRIVE_SPEED, 18, 18, 5.0); // Drive Forward

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Encoder", leftDrive.getCurrentPosition());
            telemetry.addData("Right Encoder", rightDrive.getCurrentPosition());
            telemetry.addData("Field Side", SIDE);
            telemetry.update();
            break;
        }
    }

    /*
    * Method to perform a relative move, based on encoder counts.
    */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (leftDrive.isBusy() && rightDrive.isBusy())) {

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // Pause after each move
        }
    }
    
    public void launch() {

        if (opModeIsActive()) {
            sleep(1000);
            motorCatapult.setPower(RELEASE);
            sleep(1000);
            motorCatapult.setPower(WIND);
            sleep(1000);
            motorCatapult.setPower(RELEASE);
            sleep(100);
            motorCatapult.setPower(0);
        }
    }
}
