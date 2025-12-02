package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccDrive {

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    public double drivePower = 1.0;

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
            if (drivePower == 1.00) {
                drivePower = 0.5;
            } else {
                drivePower = 1.00;
            }
        }

        // Manual control only
        double forward = gamepad.left_stick_y * drivePower;
        double turn = -gamepad.right_stick_x * drivePower;
        double strafe = -gamepad.left_stick_x * drivePower;

        setMecanum(forward, turn, strafe);

        telemetry.addData("Drive Power", drivePower);
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

}
