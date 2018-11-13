package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem{
    public Intake(HardwareMap hardwareMap) {}

    @Override
    public TelemetryPacket updateSubsystem() {
        return new TelemetryPacket(); // TODO: implemet intake class
    }
}
