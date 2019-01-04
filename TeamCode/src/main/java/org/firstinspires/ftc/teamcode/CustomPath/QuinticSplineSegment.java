package org.firstinspires.ftc.teamcode.CustomPath;

import com.acmerobotics.roadrunner.Vector2d;

public class QuinticSplineSegment extends ParametricCurve {
    class Waypoint {
        private double x, y, dx, dy, d2x, d2y;
        Waypoint(double x, double y, double dx, double dy, double d2x, double d2y) {
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.d2x = d2x;
            this.d2y = d2y;
        }

        Vector2d pos() { return new Vector2d(x, y); }
        Vector2d deriv() { return new Vector2d(dx, dy); }
        Vector2d secondDeriv() { return new Vector2d(d2x, d2y); }
    }
    QuinticPolynomial x, y;
    double length = 0.0;
    InterpolatingTreeMap arcLengthSamples = new InterpolatingTreeMap();
    static final double NUM_SAMPLES = 1000;

    public QuinticSplineSegment(Waypoint start, Waypoint end) {
        x = new QuinticPolynomial(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x);
        y = new QuinticPolynomial(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y);

        double dx = 1 / NUM_SAMPLES;
        double lastIntegrand = 0;
        for (int i = 1; i <= NUM_SAMPLES; i++) {
            double t = i * dx;
            Vector2d deriv = parametricDeriv(t);
            double integrand = Math.sqrt(deriv.getX() * deriv.getX() + deriv.getY() * deriv.getY()) * dx;
            length += (integrand + lastIntegrand) / 2.0;
            lastIntegrand = integrand;

            arcLengthSamples.put(length, t);
        }
    }



    @Override
    public Vector2d parametricDeriv(double t) { return new Vector2d(x.deriv(t), y.deriv(t)); }

    @Override
    public double displacementToParamter(double displacement) { return arcLengthSamples.getInterpolated(displacement); } // we want to get the paramter after calculating the arc length traversed

    @Override
    public double parameterDeriv(double t) {
        Vector2d deriv = parametricDeriv(t);
        return 1.0 / Math.sqrt(deriv.getX() * deriv.getX() + deriv.getY() * deriv.getY());
    }

    @Override
    public Vector2d parametricGet(double t) { return new Vector2d(x.get(t), y.get(t)); }

    public double getLength() { return length; }


}
