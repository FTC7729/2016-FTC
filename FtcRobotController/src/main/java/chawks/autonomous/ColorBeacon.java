package chawks.autonomous;


import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Beacon.BeaconAnalysis;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import chawks.autonomous.StrafeController.StrafeDirection;
import chawks.hardware.Dutchess;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by jacqu on 1/26/2017.
 */

@Autonomous(name="Camera(WeAreAlwaysWatching)", group="Pushbot")
public class ColorBeacon extends LinearVisionOpMode {

    private final int CAMERA_WIDTH = 900;
    private final int CAMERA_HEIGHT = CAMERA_WIDTH / 12 * 9;
    private final Size CAMERA_SIZE = new Size(CAMERA_WIDTH, CAMERA_HEIGHT);

    public Dutchess robot = new Dutchess();

    private Thread strafeThread;
    private StrafeController strafeController;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVision();

        /** wait for op-mode start */
        waitForStart();

        /** main loop */
        strafeController = new StrafeController(robot, 0.2f);
        strafeThread = new Thread(strafeController);
        strafeThread.start();
        strafeIntoPosition();
        telemetry.addLine(strafeController.toString());
        strafeController.stop();
    }

    /**
     * Returns true when robot has been strafed into position.
     *
     * @return true if robot is in position
     */
    private boolean strafeIntoPosition() {
        for (; ; ) {
            if (!opModeIsActive()) {
                return false;
            }
            telemetry.clear();

            // let's take a look at the beacon
            BeaconAnalysis beaconAnalysis = beacon.getAnalysis();
            telemetry.addLine(beaconAnalysis.toString());
            telemetry.addLine(strafeController.toString());

            // make adjustments based upon what we see
            double confidence = beaconAnalysis.getConfidence();
            if (confidence > .65) {
                boolean isRightBlue = beaconAnalysis.isRightBlue();
                boolean isLeftRed = beaconAnalysis.isLeftRed();

                if (isRightBlue && isLeftRed) {
                    strafeController.setDirection(StrafeDirection.LEFT);
                } else if (!isRightBlue && !isLeftRed) {
                    strafeController.setDirection(StrafeDirection.RIGHT);
                } else {
                    strafeController.setDirection(StrafeDirection.STANDSTILL);
                }
            } else {
                strafeController.setDirection(StrafeDirection.STANDSTILL);
            }
        }
    }

    private void initializeVision() throws InterruptedException {
        waitForVisionStart();

        /** Set Camera : PRIMARY == FRONT_FACING : SECONDARY == SELFIE_CAM */
        this.setCamera(Cameras.PRIMARY);

        /** Set Frame Size : LARGER == MORE_ACCURATE/SLOWER : SMALLER = LESS_ACCURATE/FASTER */
        this.setFrameSize(CAMERA_SIZE);

        /** Enable extensions below */
        enableExtensions();

        /** Beacon Analysis Method : COMMAND/CONTROL CLICK "AnalysisMethod" TO VIEW OTHER RENDERS */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         * USE THIS PIC TO UNDERSTAND TOLERANCE  http://sensing.konicaminolta.us/images/blogImages/defining-color-tolerances.png
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /** Set to true if you are using a secondary camera : not front facing */
        rotation.setIsUsingSecondaryCamera(true);

        /** this is global auto rotate : disable for normal uses */
        rotation.disableAutoRotate();

        /** force phone into orientation or to autorotate */
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);

        /** set camera conditions */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
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

