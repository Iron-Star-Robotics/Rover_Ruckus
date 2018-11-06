package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.LinkedHashMap;
import java.util.List;

public class CSVWriter implements OpModeManagerNotifier.Notifications {
    private LinkedHashMap<String, String> map;
    private List<String> header;
    private PrintStream printStream;
    private OpModeManagerImpl opModeManager;

    public CSVWriter(File file) {
        map = new LinkedHashMap<>();
        try {
            printStream = new PrintStream(file);
        } catch (IOException e) {
            e.printStackTrace();
        }
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    public void put(String key, Object value) {
        map.put(key, value.toString());
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
// close
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}
