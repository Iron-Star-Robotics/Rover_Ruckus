package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import static java.lang.Math.abs;

public class PIDController {
    private PIDCoefficients pid;
    private double errorSum,lastError;
    private double lastUpdateTimestamp = Double.NaN;

    private double kV, kA, kStatic;


    private double targetPosition = 0;

    private boolean inputBounded, outputBounded;

    private double maxInput, minInput, minOutput, maxOutput;

    public PIDController(PIDCoefficients pid, double kV, double kA, double kStatic) {
        this.pid = pid;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }

    public PIDController(PIDCoefficients pid) {
        this.pid = pid;
    }

    public void setTargetPosition(double targ) {
        targetPosition = targ;
    }


    private double getError(double position) {
        double error = targetPosition - position;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (abs(error) > inputRange / 2.0) {
                error -= Math.signum(error) * inputRange;
            }
        }
        return error;
    }

    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    public double update(double position, double velocity, double acceleration) {
        double currentTime = System.nanoTime() / 1e9;
        double error = getError(position);

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

            double baseOutput = pid.kP * error + pid.kI * errorSum + pid.kD * (errorDeriv - velocity) + kV * velocity + kA * acceleration;
            double output = abs(baseOutput) > 1e-4 ? baseOutput + Math.signum(baseOutput) * kStatic : 0.0;

            if (outputBounded) {
                return Math.max(minOutput, Math.min(output, maxOutput));
            } else {
                return output;
            }

        }
    }



    public void updatePIDCoeffecients(double kP, double kI, double kD) {
        pid.updateCoefficients(kP, kI, kD);
    }

    // reset the controllers integral sum
    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTimestamp = Double.NaN;
    }

    public double getLastError() {
        return lastError;
    }





}
