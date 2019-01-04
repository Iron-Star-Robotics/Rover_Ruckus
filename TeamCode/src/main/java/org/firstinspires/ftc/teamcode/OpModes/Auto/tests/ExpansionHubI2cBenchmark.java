package org.firstinspires.ftc.teamcode.OpModes.Auto.tests;

import com.qualcomm.hardware.BuildConfig;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.internal.system.Misc;


import android.content.Context;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.internal.system.Misc;

/**
 * Created by rbrott to test results of removing read window for i2c reads
 */

@TeleOp(name = "Expansion Hub I2C Benchmark")
public class ExpansionHubI2cBenchmark extends LinearOpMode {
    public static final int TRIALS = 250;

    private static class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private String getFtcSdkVersion() {
        try {
            Context ctx = hardwareMap.appContext;
            PackageInfo pInfo = ctx.getPackageManager().getPackageInfo(ctx.getPackageName(), 0);
            return pInfo.versionName;
        } catch (PackageManager.NameNotFoundException e) {
            // do nothing
        }
        return "?";
    }

    private static String getConciseLynxFirmwareVersion(LynxModule module) {
        String rawVersion = module.getFirmwareVersionString();
        String[] parts = rawVersion.split(" ");
        StringBuilder versionBuilder = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            String part = parts[3 + 2*i];
            if (i == 2) {
                versionBuilder.append(part);
            } else {
                versionBuilder.append(part, 0, part.length() - 1);
                versionBuilder.append(".");
            }
        }
        return versionBuilder.toString();
    }

    private int getLynxI2cVersion(LynxModule module) {
        LynxI2cDeviceSynch defaultLynxI2cDevice =
                LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, module, 0);
        if (defaultLynxI2cDevice instanceof LynxI2cDeviceSynchV1) {
            return 1;
        } else if (defaultLynxI2cDevice instanceof LynxI2cDeviceSynchV2) {
            return 2;
        } else {
            return -1;
        }
    }

    private static double getCurrentTime() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private static MovingStatistics benchmarkOperation(Func func, int trials) {
        MovingStatistics statistics = new MovingStatistics(trials);
        for (int i = 0; i < trials; i++) {
            double startTime = getCurrentTime();
            func.value();
            double elapsedTime = getCurrentTime() - startTime;
            statistics.add(elapsedTime);
        }
        return statistics;
    }

    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule module = hardwareMap.getAll(LynxModule.class).iterator().next();

        telemetry.setAutoClear(false);
        telemetry.addLine("REV Expansion Hub I2C Benchmark");
        telemetry.addLine("Press start to begin...");
        telemetry.update();

        waitForStart();

        int i2cVersion = getLynxI2cVersion(module);

        telemetry.clearAll();
        telemetry.addLine("SDK Version: " + getFtcSdkVersion());
        telemetry.addLine("ExH Version: " + getConciseLynxFirmwareVersion(module));
        telemetry.addLine("I2C Version: " + i2cVersion);
        telemetry.update();

        LynxEmbeddedIMU imu;

        // IMU v1 + read window
        imu = new LynxEmbeddedIMU(new I2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV1(hardwareMap.appContext, module, 0), true
        ));
        imu.initialize(new BNO055IMU.Parameters());

        telemetry.addLine("IMU (v1 + rw): " +
                formatResults(benchmarkOperation(imu::getAngularOrientation, TRIALS)));
        telemetry.update();

        imu.close();
        imu.resetDeviceConfigurationForOpMode();
        sleep(1000);

        // IMU v2 + read window
        if (i2cVersion == 2) {
            imu = new LynxEmbeddedIMU(new I2cDeviceSynchImplOnSimple(
                    new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
            ));
            imu.initialize(new BNO055IMU.Parameters());

            telemetry.addLine("IMU (v2 + rw): " +
                    formatResults(benchmarkOperation(imu::getAngularOrientation, TRIALS)));
            telemetry.update();

            imu.close();
            imu.resetDeviceConfigurationForOpMode();
            sleep(1000);
        } else {
            telemetry.addLine("IMU (v2 + rw): unsupported");
            telemetry.update();
        }

        // IMU v1 - read window
        imu = new LynxEmbeddedIMU(new BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV1(hardwareMap.appContext, module, 0), true
        ));
        imu.initialize(new BNO055IMU.Parameters());

        telemetry.addLine("IMU (v1 - rw): " +
                formatResults(benchmarkOperation(imu::getAngularOrientation, TRIALS)));
        telemetry.update();

        imu.close();
        imu.resetDeviceConfigurationForOpMode();
        sleep(1000);

        // IMU v2 - read window
        if (i2cVersion == 2) {
            imu = new LynxEmbeddedIMU(new BetterI2cDeviceSynchImplOnSimple(
                    new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
            ));
            imu.initialize(new BNO055IMU.Parameters());

            telemetry.addLine("IMU (v2 - rw): " +
                    formatResults(benchmarkOperation(imu::getAngularOrientation, TRIALS)));
            telemetry.update();

            imu.close();
            imu.resetDeviceConfigurationForOpMode();
            sleep(1000);
        } else {
            telemetry.addLine("IMU (v2 - rw): unsupported");
            telemetry.update();
        }

        // finish up
        telemetry.addLine("Done!");
        telemetry.update();

        while (opModeIsActive());
    }
}