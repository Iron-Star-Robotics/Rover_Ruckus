package org.firstinspires.ftc.teamcode.CustomPath;

public class ConstrainedState {
    private double maxVel, maxAcc, displacement;
    public ConstrainedState() {

    }

    public double getDisplacement() {
        return displacement;
    }

    public double getMaxAcc() {
        return maxAcc;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public void setMaxVel(double maxVel) { this.maxVel = maxVel; }
    public void setMaxAcc(double maxAcc) { this.maxAcc = maxAcc; }
    public void setDisplacement(double displacement) { this.displacement = displacement; }
}
