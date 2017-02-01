package chawks.autonomous;


import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import chawks.hardware.Dutchess;

/**
 * Created by jacqu on 1/26/2017.
 */

//@Autonomous(name="CameraLook", group="Pushbot")
public class ColorBeacon extends LinearVisionOpMode {


    private final Dutchess robot = new Dutchess();
    int frameCount = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            cameraInit();
            //Wait for the match to begin
            waitForStart();

            //Main loop
            //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
            //This loop will exit once the opmode is closed
            while (opModeIsActive()) {
                //Log a few things
                telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
                // telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
                telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
                // telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
                // telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
                // telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
                // telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
                //  telemetry.addData("Frame Counter", frameCount);

                //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
                //Vision will run asynchronously (parallel) to any user code so your programs won't hang
                //You can use hasNewFrame() to test whether vision processed a new frame
                //Once you copy the frame, discard it immediately with discardFrame()

                if (hasNewFrame()) {
                    //Get the frame
                    Mat rgba = getFrameRgba();
                    Mat gray = getFrameGray();

                    //Discard the current frame to allow for the next one to render
                    discardFrame();

                    //Do all of your custom frame processing here
                    //For this demo, let's just add to a frame counter
                    frameCount++;
                }

                //Wait for a hardware cycle to allow other processes to run
                waitOneFullHardwareCycle();


            }

        } catch (Exception e) {
            e.printStackTrace();
        }

    }

        public void cameraInit() throws InterruptedException {


        waitForVisionStart();

        /**
         * Set the camera used for detection
         * PRIMARY = Front-facing, larger camera
         * SECONDARY = Screen-facing, "selfie" camera :D
         **/
        this.setCamera(Cameras.PRIMARY);

        /**
         * Set the frame size
         * Larger = sometimes more accurate, but also much slower
         * After this method runs, it will set the "width" and "height" of the frame
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         * Enable extensions. Use what you need.
         * If you turn on the BEACON extension, it's best to turn on ROTATION too.
         */
        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

        /**
         * Set the beacon analysis method
         * Try them all and see what works!
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        /**
         * Set the rotation parameters of the screen
         * If colors are being flipped or output appears consistently incorrect, try changing these.
         *
         * First, tell the extension whether you are using a secondary camera
         * (or in some devices, a front-facing camera that reverses some colors).
         *
         * It's a good idea to disable global auto rotate in Android settings. You can do this
         * by calling disableAutoRotate() or enableAutoRotate().
         *
         * It's also a good idea to force the phone into a specific orientation (or auto rotate) by
         * calling either setActivityOrientationAutoRotate() or setActivityOrientationFixed(). If
         * you don't, the camera reader may have problems reading the current orientation.
         */
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE); // ====  Maybe set to landscape because I have no idea what im doing here

        /**
         * Set camera control extension preferences
         *
         * Enabling manual settings will improve analysis rate and may lead to better results under
         * tested conditions. If the environment changes, expect to change these values.
         */
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();



    }

    public void colorLook() throws InterruptedException {

        while (opModeIsActive()) {
            if (beacon.getAnalysis().isBeaconFound()) {



                if (beacon.getAnalysis().isRightBlue()) {

                    robot.s1.setPosition(Servo.MAX_POSITION);

                } else if (beacon.getAnalysis().isLeftBlue()) {
                    robot.s1.setPosition(Servo.MIN_POSITION);
                }

            } else {

                encoderDrive(.6, -2.0, 1.0);

            }


            telemetry.addData("Beacon Confidence: ", beacon.getAnalysis().getConfidence());
        }


    }

    public void encoderDrive(double speed, double feetDistance, double timeoutS) throws InterruptedException {
        feetDistance /= 5;
        double leftFeet = feetDistance;
        double rightFeet = feetDistance;
        double leftBackFeet = feetDistance;
        double rightBackFeet = feetDistance;
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {
            newLeftTarget = robot.lf.getCurrentPosition() + (int) (leftFeet * COUNTS_PER_INCH);
            newRightTarget = robot.rf.getCurrentPosition() + (int) (rightFeet * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lb.getCurrentPosition() + (int) (leftBackFeet * COUNTS_PER_INCH);
            newRightBackTarget = robot.rb.getCurrentPosition() + (int) (rightBackFeet * COUNTS_PER_INCH);

            robot.lf.setTargetPosition(newLeftTarget);
            robot.rf.setTargetPosition(newRightTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION		             // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            elapsedTime.reset();
            robot.lf.setPower(Math.abs(speed));
            robot.rf.setPower(Math.abs(speed) + .05);
            robot.lb.setPower(Math.abs(speed));
            robot.rb.setPower(Math.abs(speed) + .05);
            while (opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutS) && (robot.lf.isBusy() && robot.rf.isBusy()) && !wheelsAreInPosition()) {

                // Display it for the driver.		                 // Display it for the driver.
                telemetry.addData("Path1", "Going to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path1", "Going to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Currently at %7d :%7d :%7d :%7d",
                        robot.lf.getCurrentPosition(),
                        robot.rf.getCurrentPosition(),
                        robot.lb.getCurrentPosition(),
                        robot.rb.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;		             // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION		             // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public boolean wheelsAreInPosition() {
        if (robot.rb.getCurrentPosition() <= robot.rb.getTargetPosition()) {
            return false;
        } else if (robot.lb.getCurrentPosition() <= robot.lb.getTargetPosition()) {
            return false;
        } else if (robot.lf.getCurrentPosition() <= robot.lf.getTargetPosition()) {
            return false;
        } else if (robot.rf.getCurrentPosition() <= robot.rf.getTargetPosition()) {
            return false;
        } else {
            return true;
        }
    }


    }

