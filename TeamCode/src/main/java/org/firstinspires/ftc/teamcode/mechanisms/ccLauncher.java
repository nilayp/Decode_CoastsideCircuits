package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ccLauncher {

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

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
    public final double LAUNCHER_TARGET_VELOCITY = 1750;  // Value needed to reliably shoot from back launch zone
    public final double LAUNCHER_MIN_VELOCITY = 1700;     // 50 tick tolerance for "ready to fire"

    // PIDF Tuning Variables - Adjust these for tuning
    double kP = 50.0;  // Proportional gain
    double kI = 0.0;    // Integral gain
    double kD = 0.0;    // Derivative gain
    double kF = 14.166;   // Feedforward gain

    // Declare OpMode members.

    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

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
        LAUNCHING
    }

    private LaunchState launchState;

    public void init(HardwareMap map) {

        launcher = map.get(DcMotorEx.class, "launcher");
        leftFeeder = map.get(CRServo.class, "left_feeder");
        rightFeeder = map.get(CRServo.class, "right_feeder");

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

        // Set the initial launchState to IDLE.
        launchState = LaunchState.IDLE;

    }

    public void runTeleOpLoop(Gamepad gamepad, Telemetry telemetry, ccLED left, ccLED right) {
        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad.triangle) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad.circle) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        if (launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
            left.setGreenLed();
            right.setGreenLed();

        } else {
            left.setRedLed();
            right.setRedLed();
        }

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad.rightBumperWasPressed());
        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Launcher", "Target (%.2f), Min (%.2f)", LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Launcher motorSpeed", launcher.getVelocity());
    }
    private void launch(boolean shotRequested) {
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
}
