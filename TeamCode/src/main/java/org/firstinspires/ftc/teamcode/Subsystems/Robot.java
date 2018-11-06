package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    public interface Listener {
        void onPostUpdate();
    }

    public RobotTankDriveOptimized drive;
    public Intake intake;
    public Lift lift;

    private List<Subsystem> subsystems;
    private ExecutorService subsystemUpdateExecutor, telemetryUpdateExecutor;
    private BlockingQueue<TelemetryPacket> telemetryPacketQueue;

    private List<Listener> listeners;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {

    }

}
