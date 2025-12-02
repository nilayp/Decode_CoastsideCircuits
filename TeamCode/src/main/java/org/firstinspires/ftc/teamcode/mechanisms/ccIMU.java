package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class ccIMU {
    private IMU imu;

    public void init(HardwareMap map) {
        imu = map.get(IMU.class, "imu");

        // Initialize the IMU and set the correct orientation

        IMU.Parameters parms = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parms);
        imu.resetYaw();
    }
    public double getYaw() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }
}
