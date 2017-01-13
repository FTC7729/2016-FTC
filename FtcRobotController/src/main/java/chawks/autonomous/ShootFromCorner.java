package chawks.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import chawks.hardware.DutchessConfiguration;

@Autonomous(name = "Shoot from corner", group = "Pushbot")
public class ShootFromCorner extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double FORWARD_SPEED = 0.6;
    static final double BACKWARDS_SPEED = -0.6;
    /* Declare OpMode members. */
    DutchessConfiguration robot = new DutchessConfiguration();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //  robot.lf.setPower(left);
        //  robot.rf.setPower(right);
        //  robot.lb.setPower(left);
        //  robot.rb.setPower(right);

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.lf.getCurrentPosition(),
                robot.rf.getCurrentPosition(),
                robot.lb.getCurrentPosition(),
                robot.rb.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //change this value
        robot.spinMotor.setPower(0.1);
        sleep(250);
        robot.spinMotor.setPower(0.2);
        sleep(250);
        robot.spinMotor.setPower(0.3);
        sleep(250);
        robot.spinMotor.setPower(0.4);
        sleep(250);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // test 1: Drive is too long - we went all the way from starting side to the the wall all the way across
        // Turning Right went to far, did not go 90 degrees that we expected
        //encoderDrive(DRIVE_SPEED,  48,  48, 48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        // encoderDrive(DRIVE_SPEED, -24, -24, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        // test 2: driving forward - it went forward but didnt' reach the big ball
        //for test 3 make the robot go farther and modify the angle at which the robot turns
        //encoderDrive(DRIVE_SPEED,  12,  12, 12,  12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // encoderDrive(TURN_SPEED,   10, -10, 10, -10, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -12, -12, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        // test 3: it went forward turned and knocked off the big ball we took out the reversing because we didn't want it to do that and we want to focus on it going forward
        //for test 4 we will not rotate because we wnt the robot to go on the middle platform
        //encoderDrive(DRIVE_SPEED,  24,  24, 24,  24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   8, -8, 8, -8, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -12, -12, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        //test 4: it turned slightly to the right at the beggining instad of going straight and will look in code to see if it can be fixed and when we have the other robot this problem may not exist
        //encoderDrive(DRIVE_SPEED,  30, 30, 30, 30, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   8, -8, 8, -8, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -12, -12, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        //     robot.s2.setPower(-0.5);

        launchBall(-1.0, 0.8, 5.0); //servo speed, spiinner speed, timeout

        //encoderDrive(DRIVE_SPEED,  30, 30, 30, 30, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // encoderDrive(TURN_SPEED,   8, -8, 8, -8, 4.0);   // Turn Right to Position : newLeftTarget,  newRightTarget, newLeftBackTarget, newRightBackTarget
        // encoderDrive(DRIVE_SPEED,  -8, 8, 8, -8, 5.0);  // Strafe Left/Right to get onto the Center Vortex?
        //       robot.lf.setPower(BACKWARDS_SPEED);
        //  robot.rf.setPower(FORWARD_SPEED);
        //    robot.lb.setPower(FORWARD_SPEED);
        // robot.rb.setPower(BACKWARDS_SPEED);
        //


        // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void launchBall(double speedServo, double speedSpinner, double timeoutS) {

        robot.s4.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (opModeIsActive()) {

            runtime.reset();

            robot.spinMotor.setPower(speedSpinner);

            try {
                sleep(1300);
            } catch (Exception e) {
                e.printStackTrace();
            }

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {
                robot.s4.setPower(speedServo);
                robot.s3.setPower(speedServo);
                robot.s2.setPower(speedServo);

                robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);
                try {
                    sleep(1500);
                    robot.s4.setPower(0);
                    robot.s3.setPower(-.05);
                    robot.s2.setPower(0);
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            robot.spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.s4.setPower(0);
            robot.s3.setPower(-.05);
            robot.s2.setPower(0);

        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.lf.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rf.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lb.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rb.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            robot.lf.setTargetPosition(newLeftTarget);
            robot.rf.setTargetPosition(newRightTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lf.setPower(Math.abs(speed));
            robot.rf.setPower(Math.abs(speed));
            robot.lb.setPower(Math.abs(speed));
            robot.rb.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lf.isBusy() && robot.rf.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Going to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Currently at %7d :%7d :%7d :%7d",
                        robot.lf.getCurrentPosition(),
                        robot.rf.getCurrentPosition(),
                        robot.lb.getCurrentPosition(),
                        robot.rb.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
}
