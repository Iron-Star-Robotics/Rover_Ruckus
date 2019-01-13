package org.firstinspires.ftc.teamcode.Subsystems;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingMotor;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class Robot implements OpModeManagerNotifier.Notifications{

    public static final String TAG = "Robot";
    private FtcDashboard dashboard;
    private List<CachingMotor> motors;


    public MecanumDrive drive;
    public Lift lift;
    // TODO: intake, conveyor belt, lander scorer
    private OpModeManagerImpl opModeManager;



    public interface Listener {
        void onPostUpdate();
    }


    private List<Subsystem> subsystems;
    private ExecutorService subsystemUpdateExecutor;

    private BlockingQueue<TelemetryPacket> telemetryPacketQueue;

    private List<Listener> listeners;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                TelemetryPacket packet = new TelemetryPacket();
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    Map<String, Object> telemetryData = subsystem.updateSubsystem(packet.fieldOverlay());
                    packet.putAll(telemetryData);
                    // implement csv
                }
                updateMotors();
                dashboard.sendTelemetryPacket(packet); // ftc dashboard is run in a seperate thread natively so we dont need to worry about that!
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    private Runnable telemetryUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                dashboard.sendTelemetryPacket(telemetryPacketQueue.take());
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

    };

    public Robot(OpMode opMode) {
        Telemetry telemetry = opMode.telemetry;

        RevExtensions2.init();

        telemetry.log().add("REV Extensions successfully initialized!");
        this.dashboard = FtcDashboard.getInstance();
        subsystems = new ArrayList<>();
        motors = new ArrayList<>();

        boolean allGood = true;

        try {
            drive = new MecanumDrive(this, opMode.hardwareMap);
            subsystems.add(drive);
            telemetry.log().add("Mecanum Drive loaded successfully!");
        } catch (IllegalArgumentException e) {
            telemetry.log().add("Mecanum Drive init failed with: " + e.toString());
            allGood = false;
        }

        try {
            lift = new Lift(this, opMode.hardwareMap);
            subsystems.add(lift);
            telemetry.log().add("Lift loaded successfully!");
        } catch (IllegalArgumentException e) {
            telemetry.log().add("Lift init failed with: " + e.toString());
            allGood = false;
        }
        //intake = new Intake(opMode.hardwareMap);
        //subsystems.add(intake);

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem updater");
        telemetry.log().add("Started subsystem thread!");
        telemetry.log().add("Started telemetry thread!");

        if (allGood)
            telemetry.log().add("All system initialization successful!");
        else
            telemetry.log().add("Not all systems were loaded correctly >:(");

    }

    public void addListener(Listener listener) { listeners.add(listener); }

    private void updateMotors() {
        for (CachingMotor motor: motors) {
            motor.update();
        }
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }

    }

    protected void addMotor(CachingMotor motor) {
        motors.add(motor);
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }


}
