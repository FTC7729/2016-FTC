package chawks.autonomous.vision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.ftc.resq.Beacon.BeaconAnalysis;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

import chawks.hardware.Dutchess;
import chawks.hardware.MovementController;

@Autonomous(name = "TestBeaconRange", group = "Test")
public class TestBeaconRange extends LinearVisionOpMode {

    private final int CAMERA_WIDTH = 900;
    private final int CAMERA_HEIGHT = CAMERA_WIDTH / 12 * 9;
    private final Size CAMERA_SIZE = new Size(CAMERA_WIDTH, CAMERA_HEIGHT);

    private Dutchess robot = new Dutchess();

    private MovementController movementController;
    private Thread movementThread;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeVision();
        initHardware();


        // wait for op-mode start
        waitForStart();

        checkColors();

        shutDown();
    }

    public void initHardware() {
        robot.init(hardwareMap);
        movementController = new MovementController(robot, telemetry);
        movementThread = new Thread(movementController);
        movementThread.start();
    }

    public void shutDown() {
        robot.stopAllWheels();
        movementController.stop();
    }

    private boolean checkColors() {
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

            // track best we've seen
            numMeasurements++;
            double confidence = beaconAnalysis.getConfidence();
            sumConfidence += confidence;
            if (confidence > maxConfidence) {
                maxConfidence = confidence;
            }

            // make adjustments based upon what we see
            if (confidence > minConfidence) {
                boolean isRightBlue = beaconAnalysis.isRightBlue();
                boolean isLeftRed = beaconAnalysis.isLeftRed();
                boolean isLeftBlue = beaconAnalysis.isLeftBlue();
                boolean isRightRed = beaconAnalysis.isRightRed();

                if (isRightBlue) {
                    telemetry.addLine("RIGHT IS BLUE");
                }
                if (isLeftRed) {
                    telemetry.addLine("LEFT IS RED");
                }
                if (isLeftBlue) {
                    telemetry.addLine("LEFT IS BLUE");
                }
                if (isRightRed) {
                    telemetry.addLine("RIGHT IS RED");
                }

                if (isRightBlue && isLeftRed) {
                    movementController.turn(-5);
                } else if (isLeftBlue && isRightRed) {
                    movementController.turn(5);
                }


            } else {
                // TODO: Unknown what to do when unaware of beacon
                movementController.setState(MovementController.State.WaitUntilNewMovement);
                robot.stopAllWheels();
            }
            telemetry.addLine("Beacon Red and Blue: (" + beaconAnalysis.getColorString() + ")");
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

