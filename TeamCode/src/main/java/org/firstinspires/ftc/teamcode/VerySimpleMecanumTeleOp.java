package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class VerySimpleMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            double maxDrivePower = 0.3;

            forward = forward * maxDrivePower;
            turn = turn * maxDrivePower;
            strafe = strafe * maxDrivePower;

            double frontLeftPower = (forward + turn + strafe);
            double frontRightPower = (forward - turn - strafe);
            double backLeftPower = (forward + turn - strafe);
            double backRightPower = (forward - turn + strafe);

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("frontLeftDrive", frontLeftPower);
            telemetry.addData("frontRightDrive", frontRightPower);
            telemetry.addData("backLeftDrive", backLeftPower);
            telemetry.addData("backRightDrive", backRightPower);
            telemetry.addData("forward", forward);
            telemetry.addData("turn", turn);
            telemetry.addData("Strafe", strafe);
            telemetry.update();
        }
    }
}