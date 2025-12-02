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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.ccAllianceChooser;
import org.firstinspires.ftc.teamcode.mechanisms.ccDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ccIMU;
import org.firstinspires.ftc.teamcode.mechanisms.ccLED;
import org.firstinspires.ftc.teamcode.mechanisms.ccLauncher;


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

    // Declare OpMode members.
    private ccLED led1Left = null;
    private ccLED led1Right = null;
    private ccLED led2Left = null;
    private ccLED led2Right = null;

    private ccDrive drive = null;
    private ccLauncher launcher = null;
    private ccAllianceChooser allianceChooser = null;
    private ccIMU ccimu = null;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {

        DRIVING_AWAY_FROM_AUDIENCE_WALL,
        ROTATING_FOR_LAUNCH,
        STRAFING_FOR_LAUNCH,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        STRAFING_AFTER_LAUNCH,
        ROTATING_BACK,
        DRIVE_TOWARDS_AUDIENCE_WALL,
        STRAFING_OUT_OF_LAUNCH_ZONE,
        DRIVING_AWAY_2,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {

        drive = new ccDrive();
        launcher = new ccLauncher();
        led1Left = new ccLED();
        led1Right = new ccLED();
        led2Left = new ccLED();
        led2Right = new ccLED();
        allianceChooser = new ccAllianceChooser();
        ccimu = new ccIMU();

        drive.init(hardwareMap);
        launcher.init(hardwareMap);
        ccimu.init(hardwareMap);

        led1Left.init(hardwareMap, "led1_left");
        led1Right.init(hardwareMap, "led1_right");
        led2Left.init(hardwareMap, "led2_left");
        led2Right.init(hardwareMap, "led2_right");

        /*
         * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
        autonomousState = AutonomousState.DRIVING_AWAY_FROM_AUDIENCE_WALL;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        launcher.runAutoInitLoop();
        allianceChooser.init_loop(gamepad1, telemetry, led1Left, led1Right);
        telemetry.update();
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        led1Left.setLedOff();
        led1Right.setLedOff();
        led2Left.setLedOff();
        led2Right.setLedOff();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {

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
            case DRIVING_AWAY_FROM_AUDIENCE_WALL:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */
                if(drive.startDriveDistance(200,false, telemetry)) {
                    autonomousState = AutonomousState.ROTATING_FOR_LAUNCH;
                };
                break;

            case ROTATING_FOR_LAUNCH:
                double robotRotationAngle = 15;

                if (allianceChooser.alliance == ccAllianceChooser.Alliance.RED) {
                    robotRotationAngle = robotRotationAngle * -1;
                }

                if(drive.rotate(robotRotationAngle, ccimu, telemetry)){
                    autonomousState = AutonomousState.STRAFING_FOR_LAUNCH;
                }
                break;

            case STRAFING_FOR_LAUNCH:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */

                double distance = 200;
                if (allianceChooser.alliance == ccAllianceChooser.Alliance.BLUE) {
                    distance = distance * -1;
                }

                if(drive.startDriveDistance(distance,true, telemetry)) {
                    autonomousState = AutonomousState.LAUNCH;
                };
                break;

            case LAUNCH:
                launcher.launchAuto(true);
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
                if(launcher.launchAuto(false)) {
                    launcher.shotsToFire = launcher.shotsToFire - 1;
                    if(launcher.shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        launcher.stopLauncherMotor();
                        autonomousState = AutonomousState.DRIVING_AWAY_2;
                    }
                }
                break;
            case DRIVING_AWAY_2:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */
                if(drive.startDriveDistance(200,false, telemetry)) {
                    autonomousState = AutonomousState.COMPLETE;
                };
                break;
            case STRAFING_AFTER_LAUNCH:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */

                distance = -226;
                if (allianceChooser.alliance == ccAllianceChooser.Alliance.BLUE) {
                    distance = distance * -1;
                }

                if(drive.startDriveDistance(distance, true, telemetry)) {
                    autonomousState = AutonomousState.ROTATING_BACK;
                };
                break;

            case ROTATING_BACK:
                robotRotationAngle = 0;

                if(drive.rotate(robotRotationAngle, ccimu, telemetry)){
                    autonomousState = AutonomousState.DRIVE_TOWARDS_AUDIENCE_WALL;
                }
                break;

            case DRIVE_TOWARDS_AUDIENCE_WALL:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */
                if(drive.startDriveDistance(-225,false, telemetry)) {
                    autonomousState = AutonomousState.STRAFING_OUT_OF_LAUNCH_ZONE;
                };
                break;

            case STRAFING_OUT_OF_LAUNCH_ZONE:
                /*
                 * Move the robot using timing. (We don't have encoder cables attached to the
                 * robot at this point.)
                 */

                distance = -1170.0;
                if (allianceChooser.alliance == ccAllianceChooser.Alliance.BLUE) {
                    distance = distance * -1;
                }

                if(drive.startDriveDistance(distance,true, telemetry)) {
                    autonomousState = AutonomousState.COMPLETE;
                };
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
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }
}



