package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.VuforiaCamera;

public class RRAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        VuforiaCamera camera = new VuforiaCamera();
        camera.init(dashboard);
        waitForStart();
        boolean isCameraInit = false;

        while (opModeIsActive()) {
            if (!isCameraInit) {
                camera.init(dashboard);
                isCameraInit = true;
            }
        }
    }
}
