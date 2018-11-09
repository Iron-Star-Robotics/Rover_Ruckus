package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class Lift implements Subsystem {
    public Lift(HardwareMap hardwareMap) {

    }

    public TelemetryPacket updateSubsystem() {
        return new TelemetryPacket();
    }
}
