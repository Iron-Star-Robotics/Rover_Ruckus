package org.firstinspires.ftc.teamcode.Subsystems;


import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Map;

public interface Subsystem {
    Map<String, Object> updateSubsystem(Canvas fieldOverlay);
}
