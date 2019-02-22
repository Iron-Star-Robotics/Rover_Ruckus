package org.firstinspires.ftc.teamcode.lib.localization;

import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrive;

public class MecanumLocalizer {
    MecanumDrive drive;
    private Pose2d currentPose;
    //
    public MecanumLocalizer(MecanumDrive drive) {
        this.drive = drive;
    }


}
