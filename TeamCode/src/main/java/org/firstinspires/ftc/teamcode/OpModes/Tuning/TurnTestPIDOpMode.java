package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;
/*
@Config
@Autonomous(name="turnpid")
public class TurnTestPIDOpMode extends LinearOpMode {
    Robot robot;


    double globalAngle, power=0.5, correction;
    Orientation lastAngles = new Orientation();
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0,0,0);
    PIDController pidRotate;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();
        pidRotate = new PIDController(HEADING_PID);

        waitForStart();
        while (opModeIsActive()) {
            rotate(- 1 * Math.PI / 2, power);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("headingError: ", pidRotate.getLastError());
            robot.drive.getDashboard().sendTelemetryPacket(packet);
            sleep(3000);
        }

    }

    private void rotate(double radians, double power) {
        resetAngle();
        pidRotate.reset();
        pidRotate.setInputBounds(0, Math.PI);
        pidRotate.setOutputBounds(.3, power);
        pidRotate.setTargetPosition(radians);

        // rotate until turn in complete
        if (radians < 0) {
            telemetry.log().add("turn right");
            while (opModeIsActive() && getAngle() == 0) {
                robot.drive.setMotorPowers(-power, power);
                sleep(100);
            }
            do {
                power = pidRotate.update(getAngle(), 0, 0);
                robot.drive.setMotorPowers(power, -power);
            } while (opModeIsActive() &&  Math.abs(pidRotate.getLastError()) < 0.0523599);
        } else {
            do {
                power = pidRotate.update(getAngle(), 0, 0);
                robot.drive.setMotorPowers(power, -power);
            } while (opModeIsActive() && Math.abs(pidRotate.getLastError()) < 0.0349066);
        }



        robot.drive.setMotorPowers(0,0);
        sleep(500);
        resetAngle();



    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        telemetry.log().add("" + globalAngle);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -Math.PI)
            deltaAngle += 2 * Math.PI;
        else if (deltaAngle > Math.PI)
            deltaAngle -= 2 * Math.PI;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = robot.drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }




}
*/