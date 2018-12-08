package org.firstinspires.ftc.teamcode.CustomPath;


import com.acmerobotics.roadrunner.Vector2d;
/*
public class QuinticSplineSegment extends ParametricCurve{
    private QuinticPolynomial x;
    private QuinticPolynomial y;

    /**
     * Class for representing the end point of the interpolated quintic splines
     */
/*

    class Waypoint {
        private double x,y,dx,dy,d2x,d2y;
        public Waypoint(double x, double y, double dx, double dy, double d2x, double d2y) {
            this.x = x;
            this.y = y;
            this.dx = dx;
            this.dy = dy;
            this.d2x = d2x;
            this.d2y = d2y;
        }

        Vector2d pos() { return new Vector2d(x,y); }
        Vector2d deriv() { return new Vector2d(dx, dy); }
        Vector2d secondDeriv() { return new Vector2d(d2x, d2y); }
    }



    private double length;
    private InterpolatingTreeMap arcLengthSamples = new InterpolatingTreeMap();
    public QuinticSplineSegment(Waypoint start, Waypoint end) {
        this.x = new QuinticPolynomial(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x);
        this.y = new QuinticPolynomial(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y);
        arcLengthSamples[0.0] = 0.0;
        double dx = 1.0 / 1000;
        double sum = 0;
        double lastIntegrand = 0.0;
        int i = 1;
        while (i <= 1000) {
            double t = i * dx;
            Vector2d deriv = getPosDeriv(t);
            double integrand = Math.sqrt(deriv.getX() * deriv.getX() + deriv.getY() * deriv.getY()) * dx;
            sum += (integrand + lastIntegrand) / 2.0;
            lastIntegrand = integrand;
            arcLengthSamples[sum] = t;
            i+=1;
        }
        length = sum;

    }

    Vector2d getPosVector(double t) { return new Vector2d(x.get(t), y.get(t)); }
    Vector2d getPosDeriv(double t) { return new Vector2d(x.deriv(t), y.deriv(t)); }

    public double parameterDeriv(double t) {
        Vector2d deriv = getPosDeriv(t);
        return 1.0 / Math.sqrt(deriv.getX() * deriv.getX() + deriv.getY() * deriv.getY());
    }

    public double parameterSecondDeriv(double t) {

    }






}*/
