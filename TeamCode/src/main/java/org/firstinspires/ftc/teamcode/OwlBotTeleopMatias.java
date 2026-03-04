package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="OwlBotTeleopMatias", group="Starterbot")
public class OwlBotTeleopMatias extends LinearOpMode {
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotorEx catapult;

    private Servo shooter;
    private CRServo shooter2;
    private CRServo middleFeeder;
    private CRServo intakeFeeder;

    private Servo turret;

    // Turret preset positions (in degrees)
    double turretCenter = 95.0;
    double turretLeft = 160.0;    // Adjust these values for your specific positions
    double turretRight = 30.0;    // Adjust these values for your specific positions
    double turretCurrent = 95.0;

    boolean lastLeftBumper = false;

    double teleOpDrivePower = 0.75;

    long pushTime = 250;
    long totalCycle = 400;

    boolean lastA = false;
    boolean isFiring = false;
    long fireStartTime = 0;
    long lastShotTime = 0;
    long shotCooldown = 0;

    double shooterForward = 1.0;
    double shooterBack = 0.5;

    private static final double MIN_VELOCITY_THRESHOLD = 4000;
    private static final double TARGET_VELOCITY = 4500;
    private static final double TICKS_PER_REVOLUTION = 28.0;

    private boolean catapultOn = false;
    private boolean lastX = false;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        catapult = hardwareMap.get(DcMotorEx.class, "launcher");
        catapult.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter = hardwareMap.servo.get("endFeeder");
        shooter2 = hardwareMap.crservo.get("shooter2");
        middleFeeder = hardwareMap.crservo.get("middleFeeder");
        turret = hardwareMap.servo.get("turret");

        intakeFeeder = hardwareMap.crservo.get("intakeFeeder");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * teleOpDrivePower;
            double turn = -gamepad1.right_stick_x * teleOpDrivePower;
            double strafe = -gamepad1.left_stick_x * teleOpDrivePower * 1.2;

            if (gamepad1.left_bumper && !lastLeftBumper) {
                teleOpDrivePower = (teleOpDrivePower == 1.0) ? 0.5 : 1.0;
            }
            lastLeftBumper = gamepad1.left_bumper;

            double frontLeftPower = (forward + turn - strafe);
            double frontRightPower = (forward - turn - strafe);
            double backLeftPower = (forward + turn + strafe);
            double backRightPower = (forward - turn + strafe);

            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Turret positional control with D-pad
            if (gamepad1.dpad_left) {
                turretCurrent = turretLeft;
            } else if (gamepad1.dpad_right) {
                turretCurrent = turretRight;
            } else if (gamepad1.dpad_up) {
                turretCurrent = turretCenter;
            }
            // Optional: Add dpad_down for another position if needed

            // Ensure turret position is within valid range
            turretCurrent = Math.max(0.0, Math.min(190.0, turretCurrent));
            turret.setPosition(turretCurrent / 190.0);

            double currentVelocityTicksPerSec = catapult.getVelocity();
            double currentVelocityRPM = -(currentVelocityTicksPerSec / TICKS_PER_REVOLUTION) * 60.0;
            boolean canShoot = currentVelocityRPM >= MIN_VELOCITY_THRESHOLD;

            long now = System.currentTimeMillis();

            if (gamepad1.right_bumper && !isFiring && canShoot && catapultOn && (now - lastShotTime > shotCooldown)) {
                isFiring = true;
                fireStartTime = now;
                lastShotTime = now;
            }

            long elapsed = now - fireStartTime;

            if (isFiring && elapsed < pushTime) {
                shooter.setPosition(shooterForward);
                shooter2.setPower(-1.0);
            } else {
                shooter.setPosition(shooterBack);
                shooter2.setPower(0.0);
            }


            if (isFiring && elapsed >= totalCycle) {
                isFiring = false;
            }

            if (gamepad1.right_trigger > 0.3) {
                intakeFeeder.setPower(gamepad1.right_trigger);
                middleFeeder.setPower(-gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.3) {
                intakeFeeder.setPower(-gamepad1.left_trigger);
                middleFeeder.setPower(gamepad1.left_trigger);
            } else {
                intakeFeeder.setPower(0.0);
                middleFeeder.setPower(0.0);
            }

            if (gamepad1.x) {
                if (!lastX) {
                    if (catapultOn) {
                        catapultOn = false;
                    } else {
                        catapultOn = true;
                    }
                }
                lastX = true;
            } else {
                lastX = false;
            }

            double targetTicksPerSecond = (TARGET_VELOCITY / 60.0) * TICKS_PER_REVOLUTION;

            if (catapultOn) {
                catapult.setVelocity(-targetTicksPerSecond);
            } else {
                catapult.setVelocity(0);
            }

            telemetry.addData("frontLeftDrive", frontLeftPower);
            telemetry.addData("frontRightDrive", frontRightPower);
            telemetry.addData("backLeftDrive", backLeftPower);
            telemetry.addData("backRightDrive", backRightPower);
            telemetry.addData("forward", forward);
            telemetry.addData("turn", turn);
            telemetry.addData("strafe", strafe);
            telemetry.addData("Catapult On", catapultOn);
            telemetry.addData("Catapult RPM", currentVelocityRPM);
            telemetry.addData("Flywheel Ready", canShoot ? "YES" : "SPINNING UP");
            telemetry.addData("Turret Degrees", turretCurrent);
            telemetry.addData("Drive Power", teleOpDrivePower);
            telemetry.addData("Is Firing", isFiring);
            telemetry.update();
        }
    }
}