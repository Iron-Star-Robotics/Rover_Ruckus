package org.firstinspires.ftc.teamcode.CustomPath;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

public class TimedTrajectory {
    private boolean reversed;
    private double startVelo, endVelo, maxVelo, maxAbsAccel;
    InterpolatingTreeMap samples;
    public TimedTrajectory(boolean reversed, double startVelo, double endVelo, double maxVelo, double maxAbsAccel, InterpolatingTreeMap samples) {
        this.reversed = reversed;
        this.startVelo = startVelo;
        this.endVelo = endVelo;
        this.maxVelo = maxVelo;
        this.maxAbsAccel = maxAbsAccel;
    }




}
