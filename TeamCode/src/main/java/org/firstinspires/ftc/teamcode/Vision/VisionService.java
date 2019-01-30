package org.firstinspires.ftc.teamcode.Vision;

import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

public class VisionService {
    private static String VUFORIA_LICENSE_KEY = "Aa7V0cb/////AAABmTsi2Ahmh0zWjzLxzU0mNG9cJ6TYhep67J8OK80SO/2+0+PwMBRgqCl9xE09y/Jp+8+zHWLX5onVDZfIpo1pVh7aivHulBtB7rXr5WLmzEWVRpPhNAnxRc1SVZ9H9cPmFRkIddugcPOc9W6+S+wHZGCg2Biv7Mvyao4uf8PJ8k5mNNJP1CWWE+4k86DkQjPGO8aFatDaDJOwTRQbr7niDGTcEdGU6pDbMWUaZItsMd7somOfiTZKMRSBn1ztRM/9uyzApzhLP4vB8cwHvD1RRdcK4D1Vk4Q6RrCIZRJd2jzfdoBzvAjora7yQAVMQkfwtpZnepTe29hRYLh8RXHt7fwDrWpZkMk74VNdkX7cM9mZ";
    Dogeforia df;
    GoldAlignDetector detector;

    public enum GoldPos {
        LEFT,
        CENTER,
        RIGHT
    }

    public VisionService(GoldAlignDetector detector) {
        VuforiaLocalizer.Parameters vuforiaParams; vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        this.detector = detector;
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        df = new Dogeforia(vuforiaParams);
        df.setDogeCVDetector(detector);
    }

    public void start() {
        df.enableDogeCV();
        df.start(); // start vision thread
    }

    public GoldPos getGoldLocation() {
        if (detector.getAligned())
            return GoldPos.CENTER;
        else if (detector.isFound())
            return GoldPos.LEFT;
        return GoldPos.RIGHT;
    }

    public void stop() {
        df.stop();
    }
}
