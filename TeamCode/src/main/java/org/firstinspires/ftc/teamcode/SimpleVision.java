package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import java.util.List;

@TeleOp(name = "SimpleVision", group = "Software")
@Disabled
public class SimpleVision extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor april;

    @Override
    public void runOpMode() {
        // 1) Build the AprilTag processor
        april = new AprilTagProcessor.Builder()
                // The default library is TAG36h11, which is what is used in the game.

                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        // 2) Build the VisionPortal on the webcam named "Webcam 1"
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(april)
                .build();

        telemetry.addLine("Initialized. Waiting for START...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = april.getDetections();

            if (detections.isEmpty()) {
                telemetry.addLine("No AprilTags visible");
            } else {
                // Use the first detection (or choose your own selection logic)
                AprilTagDetection d = detections.get(0);

                telemetry.addData("Tag ID", d.id);
                // ftcPose units:
                //  - x, y, z in METERS (given setTagSize is correct)
                //  - yaw, pitch, roll in DEGREES
                telemetry.addData("X (m)", "%.3f", d.ftcPose.x);
                telemetry.addData("Y (m)", "%.3f", d.ftcPose.y);
                telemetry.addData("Z (m)", "%.3f", d.ftcPose.z);
                telemetry.addData("Range (m)", "%.3f", d.ftcPose.range);
                telemetry.addData("Yaw (deg)", "%.1f", d.ftcPose.yaw);
                telemetry.addData("Pitch (deg)", "%.1f", d.ftcPose.pitch);
                telemetry.addData("Roll (deg)", "%.1f", d.ftcPose.roll);

                // Optional extras you may find handy:
                // telemetry.addData("Range (m)", "%.3f", d.ftcPose.range);
                // telemetry.addData("Bearing (deg)", "%.1f", d.ftcPose.bearing);
                // telemetry.addData("Elevation (deg)", "%.1f", d.ftcPose.elevation);
            }

            telemetry.update();

            // Yield a little time so the portal can process frames smoothly
            sleep(10);
        }

        // Clean up (optional but tidy)
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}