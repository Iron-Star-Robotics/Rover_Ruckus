package org.firstinspires.ftc.teamcode.CustomPath;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

public class QuinticPolynomial {
    private double a,b,c,d,e,f;
    private static final double[][] COEFFS = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
            {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
            {0.0, 0.0, 0.0, 2.0, 0.0, 0.0},
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
            {5.0, 4.0, 3.0, 2.0, 1.0, 0.0},
            {20.0, 12.0, 6.0, 2.0, 0.0, 0.0}
    };

    /**
     * Quintic polynomial interpolated according to the provided derivatives.
     *
     * @param start start value
     * @param startDeriv start derivative
     * @param startSecondDeriv start second derivative
     * @param end end value
     * @param endDeriv end derivative
     * @param endSecondDeriv end second derivative
     */

    public QuinticPolynomial(double start, double startDeriv, double startSecondDeriv, double end, double endDeriv, double endSecondDeriv) {
        double[][] targetArray = {{start, startDeriv, startSecondDeriv, end, endDeriv, endSecondDeriv}};
        RealMatrix target = MatrixUtils.createRealMatrix(targetArray);
        DecompositionSolver solver = new LUDecomposition(COEFF_MATRIX).getSolver();
        RealMatrix coeffs = solver.solve(target);

        this.a = coeffs.getEntry(0, 0);
        this.b = coeffs.getEntry(1, 0);
        this.c = coeffs.getEntry(2, 0);
        this.d = coeffs.getEntry(3, 0);
        this.e = coeffs.getEntry(4, 0);
        this.f = coeffs.getEntry(5, 0);
    }

    private static final RealMatrix COEFF_MATRIX = MatrixUtils.createRealMatrix(COEFFS);

    // returns value of the polynomial at a given time
    public double get(double t) {
        return ((a*t + b) * (t*t*t*t) + c * (t*t*t) + d * (t*t) + e * t + f);
    }

    // returns derivative of the polynomial at a given time
    public double deriv(double t) {
        return (5*a*t + 4*b) * (t*t*t) + (3*c*t + 2*d) * t + e;
    }



}
