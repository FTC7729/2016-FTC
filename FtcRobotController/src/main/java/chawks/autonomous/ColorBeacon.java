package chawks.autonomous;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import chawks.hardware.Dutchess;

/**
 * Created by jacqu on 1/26/2017.
 */

//@Autonomous(name="CameraLook", group="Pushbot")
public class ColorBeacon extends LinearVisionOpMode {

    private final int WIDTH = 150;
    private final int HEIGHT = WIDTH / 12 * 9;
    private final int SCALE = 6;
    private final Size SIZE = new Size(WIDTH * SCALE, HEIGHT * SCALE);

    @Override
    public void runOpMode() throws InterruptedException {

        waitForVisionStart();

        /** Set Camera : PRIMARY == FRONT_FACING : SECONDARY == SELFIE_CAM */
        this.setCamera(Cameras.PRIMARY);

        /** Set Frame Size : LARGER == MORE_ACCURATE/SLOWER : SMALLER = LESS_ACCURATE/FASTER */
        this.setFrameSize(SIZE);

        /** Enable extensions below */
        enableExtensions();

        /** Beacon Analysis Method : COMMAND/CONTROL CLICK "AnalysisMethod" TO VIEW OTHER RENDERS */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         * USE THIS PIC TO UNDERSTAND TOLERANCE  http://sensing.konicaminolta.us/images/blogImages/defining-color-tolerances.png
         */
        beacon.setColorToleranceRed(1);
        beacon.setColorToleranceBlue(-1);

        /** Set to true if you are using a secondary camera : not front facing */
        rotation.setIsUsingSecondaryCamera(false);

        /** this is global auto rotate : disable for normal uses */
        rotation.disableAutoRotate();

        /** force phone into orientation or to autorotate */
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);

        /** set camera conditions */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        /** wait for start */
        waitForStart();

        /** main loop */
        while (opModeIsActive()) {
            Mat rgba = getFrameRgba();
            if (rgba.get(0,0)[0] < rgba.get(1,0)[0]) {
                // TODO: RED IS ON THE LEFT
            } else if (rgba.get(0,0)[0] > rgba.get(1,0)[0]) {
                // TODO: RED IS ON THE RIGHT
            }
        }

    }

    public void enableExtensions() {

        /** Enable Extension : beacon detection */
        enableExtension(Extensions.BEACON);

        /** Enable Extension : automatic screen rotation correction */
        enableExtension(Extensions.ROTATION);

        /** Enable Extension : manual camera control */
        enableExtension(Extensions.CAMERA_CONTROL);

    }

}

