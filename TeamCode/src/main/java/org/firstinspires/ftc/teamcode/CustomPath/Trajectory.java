package org.firstinspires.ftc.teamcode.CustomPath;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;

public class Trajectory {
    List<ParametricCurve> curves;
    boolean reversed = false;
    public Trajectory(List<ParametricCurve> curves) {
        this.curves = curves;
    }


    public double getLength() {
        int sum = 0;
        for (int i = 0; i < curves.size(); i++)
            sum+=curves.get(i).getLength();
        return sum;
    }

    public Pose2d get(double displacement) {
        double remainingDis = displacement;
        for (ParametricCurve curve : curves) {
            if (remainingDis <= curve.getLength()) {
                Vector2d point;
                if (reversed)
                    point = curve.get(displacement - remainingDis);
                else
                    point = curve.get(displacement);
            /*
                TODO: add heading interpolater
             */

                return new Pose2d(point.getX(), point.getY(), 0.0);
            }

            remainingDis -= curve.getLength();
        }
        Vector2d endVec =  curves.get(curves.size() - 1).end();
        return new Pose2d(endVec.getX(), endVec.getY(), 0.0); // again still need to do heading interpolator
    }

    public Pose2d deriv(double displacement) {
        double remainingDis = displacement;
        for (ParametricCurve curve : curves) {
            if (remainingDis <= curve.getLength()){
                Vector2d deriv;
                if (reversed)
                    deriv = curve.deriv(displacement - remainingDis).times(-1);
                else
                    deriv = curve.deriv(displacement);

                return new Pose2d(deriv.getX(), deriv.getY(), 0.0);
            }
            remainingDis -= curve.getLength();
        }

        Vector2d endVec =  curves.get(curves.size() - 1).end();
        return new Pose2d(endVec.getX(), endVec.getY(), 0.0);
    }


}
