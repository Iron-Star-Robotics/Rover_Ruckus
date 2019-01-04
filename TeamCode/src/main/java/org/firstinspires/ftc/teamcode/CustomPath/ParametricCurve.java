package org.firstinspires.ftc.teamcode.CustomPath;

import com.acmerobotics.roadrunner.Vector2d;

public abstract class ParametricCurve {
    public ParametricCurve() {

    }

    public Vector2d get(double displacement) {
        return parametricGet(displacementToParamter(displacement));
    }

    public Vector2d deriv(double displacement) {
        double t = displacementToParamter(displacement);
        return parametricDeriv(t).times(parameterDeriv(t));
    }

    public Vector2d start() {
        return get(0.0);
    }

    public Vector2d end() { return get(getLength()); }

    public abstract double getLength();
    public abstract double displacementToParamter(double displacement);
    public abstract double parameterDeriv(double t);
    public abstract Vector2d parametricDeriv(double t);
    public abstract Vector2d parametricGet(double t);
}
