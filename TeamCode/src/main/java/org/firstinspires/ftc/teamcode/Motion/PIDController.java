package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

import static java.lang.Math.abs;

public class PIDController {
    private PIDCoefficients coeff;
    private double errorSum,lastError;
    private double lastUpdateTimestamp = Double.NaN;

    private double kV, kA, kStatic;
    private PIDCoefficients pidCoefficients;
    private MultipleTelemetry pidTelemetry;
    private double targetPosition = 0;

    private boolean inputBounded, outputBounded;

    private double maxInput, minInput, minOutput, maxOutput;

    public PIDController(PIDCoefficients pid, MultipleTelemetry telemetry, double kV, double kA, double kStatic) {
        this.pidCoefficients = pid;
        this.pidTelemetry = telemetry;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }

    private double getError(double position, String type) {
        double error = targetPosition - position;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (abs(error) > inputRange / 2.0) {
                error -= Math.signum(error) * inputRange;
            }
        }
        pidTelemetry.addData(type, error);
        return error;
    }

    public double update(double position, String type, double velocity, double acceleration) {
        double currentTime = System.nanoTime() / 1e9;
        double error = getError(position, type);

        if (lastUpdateTimestamp == Double.NaN) {
            lastError = error;
            lastUpdateTimestamp = currentTime;
            return 0.0;
        } else {
            double dt = currentTime - lastUpdateTimestamp;
            errorSum += 0.5 * (error + lastError) * dt;
            double errorDeriv = (error - lastError) / dt;

            lastError = error;
            lastUpdateTimestamp = currentTime;

            double baseOutput = pidCoefficients.kP * error + pidCoefficients.kI * errorSum + pidCoefficients.kD * (errorDeriv - velocity) + kV * velocity + kA * acceleration;
            double output = abs(baseOutput) > 1e-4 ? baseOutput + Math.signum(baseOutput) * kStatic : 0.0;

            if (outputBounded) {
                return Math.max(minOutput, Math.min(output, maxOutput));
            } else {
                return output;
            }

        }
    }

    // reset the controllers integral sum
    private void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTimestamp = Double.NaN;
    }





}
