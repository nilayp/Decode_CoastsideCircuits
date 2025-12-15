/*
 * Copyright (c) 2025 FIRST
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
 * Neither the name of FIRST nor the names of its contributors may be used to
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.ccAllianceChooser;
import org.firstinspires.ftc.teamcode.mechanisms.ccContinuousRotationServo;
import org.firstinspires.ftc.teamcode.mechanisms.ccDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ccLED;
import org.firstinspires.ftc.teamcode.mechanisms.ccLauncher;
import org.firstinspires.ftc.teamcode.mechanisms.ccPositionalServo;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "OwlBotTeleOpManualShooter", group = "Production")
public class OwlBotTeleopManualShooter extends OpMode {

    private ccDrive drive = null;
    private ccContinuousRotationServo intake = null;
    private ccPositionalServo turret = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        drive = new ccDrive();
        intake = new ccContinuousRotationServo();
        turret = new ccPositionalServo();

        drive.init(hardwareMap);
        intake.init(hardwareMap, "intakeFeeder", telemetry);
        turret.init(hardwareMap,"turret", 200, 4, telemetry);

        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addLine("Status Initialized. Press start to run.");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

        // allianceChooser.init_loop(gamepad1, telemetry, led2Left, led2Right);
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        drive.runTeleOpLoop(gamepad1, telemetry);
        intake.teleOpLoop(gamepad1, telemetry);
        turret.teleOpLoop(gamepad1,telemetry);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}