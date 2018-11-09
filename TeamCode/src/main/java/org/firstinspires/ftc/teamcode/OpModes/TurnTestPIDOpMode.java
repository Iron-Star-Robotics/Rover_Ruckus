package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;

@Config
public class TurnTestPIDOpMode extends LinearOpMode {
    RobotTankDrive drive;
    double globalAngle, power=0.5, correction;
    Orientation lastAngles = new Orientation();
    public static PIDCoefficients HEADING_PID = new PIDCoefficients();
    PIDFController pidRotate;

    @Override
    public void runOpMode() {
        drive = new RobotTankDrive(hardwareMap);
        pidRotate = new PIDFController(HEADING_PID);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            rotate(Math.PI / 2, power);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("headingError: ", pidRotate.getLastError());
            sleep(3000);
        }

    }

    private void rotate(double radians, double power) {
        resetAngle();
        pidRotate.reset();
        pidRotate.setInputBounds(-Math.PI, Math.PI);
        pidRotate.setOutputBounds(.3, power);

        // rotate until turn in complete
        if (radians < 0) {
            while (opModeIsActive() && getAngle() == 0) {
                drive.setMotorPowers(-power, power);
                sleep(100);
            }
            do {
                power = pidRotate.update(getAngle());
                drive.setMotorPowers(power, -power);
            } while (opModeIsActive() && pidRotate.getLastError() != 0);
        } else {
            do {
                power = pidRotate.update(getAngle());
                drive.setMotorPowers(power, -power);
            } while (opModeIsActive() && pidRotate.getLastError() != 0);
        }



        drive.setMotorPowers(0,0);
        sleep(500);
        resetAngle();



    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

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
        lastAngles = drive.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }




}
