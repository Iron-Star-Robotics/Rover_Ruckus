package org.firstinspires.ftc.teamcode.Motion;

public class PIDCoefficients {
    public double kP, kI, kD;
    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void updateCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }


}
