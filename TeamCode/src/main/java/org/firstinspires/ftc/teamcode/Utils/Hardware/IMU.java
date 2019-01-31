package org.firstinspires.ftc.teamcode.Utils.Hardware;

import android.support.annotation.MainThread;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.openftc.revextensions2.ExpansionHubEx;

public class IMU {

    private double bias = 0;

    private static final double TAU = Math.PI * 2;

    public enum IMU_POS {
        VERTICAL,
        HORIZONTAL
    }

    private ExpansionHubEx hub;
    private BNO055IMU imu;

    private IMU_POS pos = IMU_POS.HORIZONTAL;

    private double cachedAngle = 0;

    public IMU(IMU_POS pos, ExpansionHubEx hub) {
        this.hub = hub;
        this.pos = pos;
    }

    public void init() {
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        if (pos == IMU_POS.VERTICAL)
            axisRemap();
    }

    public IMU() {}

    public void setBias(double radians) {
        this.bias = radians;
    }

    private void axisRemap() {
        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0b00100001; //swaps y-z, 0b00100001 is y-x, 0x6 is x-z
            // uhh idk if this is the right thing
            byte AXIS_MAP_SIGN_BYTE = 0b000; //x, y, zv

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100); //Changing modes requires a delay before doing anything else

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public double getRawAngle() {
        return cachedAngle;
    }

    public double getAngle() {
        return norm(cachedAngle);
    }

    public void update() {
        cachedAngle = imu.getAngularOrientation().firstAngle + bias;
    }

    /**
     * Returns [angle] clamped to `[-pi, pi]`.
     *
     * @param angle angle measure in radians
     */

    private double norm(double angle) {
        double newAngle = angle % TAU;

        newAngle = (newAngle + TAU) % TAU;

        if (newAngle > Math.PI)
            newAngle -= TAU;

        return newAngle;
    }


}
