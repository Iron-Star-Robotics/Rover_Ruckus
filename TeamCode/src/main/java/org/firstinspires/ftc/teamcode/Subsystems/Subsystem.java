package org.firstinspires.ftc.teamcode.Subsystems;


import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.Map;

public interface Subsystem {
    Map<String, Object> update(@Nullable Canvas fieldOverlay);
}
