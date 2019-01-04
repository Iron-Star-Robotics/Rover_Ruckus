package org.firstinspires.ftc.teamcode.Utils.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

/*
    performs a simple launching routine
 */

public class Launcher {
    private OpMode opMode;
    private Robot robot;
    private ExpansionHubEx hub;

    // some globals we will need to share across subsystems

    public Launcher(OpMode opMode) {
        RevExtensions2.init();
        this.opMode = opMode;
        hub = opMode.hardwareMap.get(ExpansionHubEx.class, "hub");
    }

    public void launch() {
        robot = new Robot(opMode, hub);
        robot.start();
    }

    public Robot getRobot() { return robot; }
}
