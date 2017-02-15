package chawks.autonomous.vision;


import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Beacon.BeaconAnalysis;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import chawks.hardware.MovementController;
import chawks.hardware.Dutchess;

public abstract class AbstractVisionOpMode extends LinearVisionOpMode {

    private final int CAMERA_WIDTH = 900;
    private final int CAMERA_HEIGHT = CAMERA_WIDTH / 12 * 9;
    private final Size CAMERA_SIZE = new Size(CAMERA_WIDTH, CAMERA_HEIGHT);

    /**
     * This is our robot
     */
    private final Dutchess robot;

    /**
     * Default speed/power applied to wheels
     */
    private final float wheelPower;

    /**
     * Manages movement of the robot
     */
    private MovementController movementController;

    /**
     * Movement controller runs as a background thread
     */
    private Thread movementThread;

    /**
     * If true, we do extra logging to telemetry
     */
    private boolean debug;

    public AbstractVisionOpMode(Dutchess robot, float wheelPower) {
        this.robot = robot;
        this.wheelPower = wheelPower;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initializeVision();

        // wait for op-mode start
        waitForStart();

        // create movement controller
        movementController = new MovementController(robot, telemetry);
        movementController.setWheelPower(wheelPower);

        // movement processed in background
        movementThread = new Thread(movementController);
        movementThread.start();

        // main loop
        strafeIntoPosition();

        // stop movement
        movementController.stop();
    }

    /**
     * Returns true when robot has been strafed into position.
     *
     * @return true if robot is in position
     */
    private boolean strafeIntoPosition() {
        double maxConfidence = 0.0;
        double sumConfidence = 0.0;
        int numMeasurements = 0;
        double minConfidence = .9;

        for (; ; ) {
            if (!opModeIsActive()) {
                return false;
            }
            telemetry.clear();

            // let's take a look at the beacon
            BeaconAnalysis beaconAnalysis = beacon.getAnalysis();
            telemetry.addLine(beaconAnalysis.toString());
            telemetry.addLine(movementController.toString());

            // track best we've seen
            numMeasurements++;
            double confidence = beaconAnalysis.getConfidence();
            sumConfidence += confidence;
            if (confidence > maxConfidence) {
                maxConfidence = confidence;
            }
            if (debug) {
                telemetry.addData("conf=%.2f", confidence);
                telemetry.addData("avg=%.2f", (sumConfidence / numMeasurements));
            }

            // make adjustments based upon what we see
            if (confidence > minConfidence) {
                boolean isRightBlue = beaconAnalysis.isRightBlue();
                boolean isLeftRed = beaconAnalysis.isLeftRed();
                boolean isLeftBlue = beaconAnalysis.isLeftBlue();
                boolean isRightRed = beaconAnalysis.isRightRed();

                if (isLeftBlue && isRightBlue) {

                } else if (isRightRed && isLeftRed) {

                }

                if (isRightBlue) {
                    // move one inch
                    movementController.strafeLeft(1.0d);
                } else {
                    // move on inch
                    movementController.strafeRight(1.0d);
                }
            } else {
                // TODO: not sure if we should stop here; maybe we backup, or change direction,
                // or keep going in last direction, until we have some confidence; need to do
                // some thinking and testing about this
                movementController.stopMoving();
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
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         * USE THIS PIC TO UNDERSTAND TOLERANCE  http://sensing.konicaminolta.us/images/blogImages/defining-color-tolerances.png
         */
        beacon.setColorToleranceRed(.8);
        beacon.setColorToleranceBlue(-.8);

        /** Set to true if you are using a secondary camera : not front facing */
        rotation.setIsUsingSecondaryCamera(false);

        /** this is global auto rotate : stop for normal uses */
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

