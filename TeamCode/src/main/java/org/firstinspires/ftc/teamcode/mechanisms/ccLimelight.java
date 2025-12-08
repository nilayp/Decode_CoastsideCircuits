package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class ccLimelight {

    private Limelight3A limelight;
    private double currentYaw = 0.0;
    Pose3D botpose = null;


    public void init(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");

        // Start the limelight so pipeline switching works properly
        limelight.start();
    }

    public void getMegaTag1Data(Telemetry telemetry) {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            botpose = llResult.getBotpose();
            telemetry.addLine("---- MegaTag1 ----");
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTx());
            telemetry.addData("Ta", llResult.getTx());
        }
    }
    public void getMegaTag2Data(ccIMU ccimu, Telemetry telemetry) {
        limelight.updateRobotOrientation(ccimu.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            botpose = llResult.getBotpose_MT2();
            telemetry.addLine("---- MegaTag2 ---- ");
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTx());
            telemetry.addData("Ta", llResult.getTx());
        }
    }

    public void switchPipelineByAlliance(ccAllianceChooser.Alliance alliance) {
        if (alliance == ccAllianceChooser.Alliance.RED) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }
    }

    public double getTx(ccIMU ccimu) {
        limelight.updateRobotOrientation(ccimu.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            botpose = llResult.getBotpose_MT2();
            return llResult.getTx();
        }
        // this code needs to return a double, even if there is no result from the
        // limelight. -360.0 seemed safe.
        return -360.0;
    }
}
