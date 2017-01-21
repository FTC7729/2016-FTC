package chawks.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import chawks.hardware.Dutchess;

public abstract class AbstractAutonomous extends LinearOpMode {
    /**
     * Robot configuration
     **/

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final Dutchess robot = new Dutchess();

    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.lf.getCurrentPosition(),
                robot.rf.getCurrentPosition(),
                robot.lb.getCurrentPosition(),
                robot.rb.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.spinMotor.setPower(0.1);
        sleep(250);

        robot.spinMotor.setPower(0.2);
        sleep(250);

        robot.spinMotor.setPower(0.3);
        sleep(250);

        robot.spinMotor.setPower(0.36);
        sleep(250);

        launchBall(-1.0, 0.8, 5.0); //servo speed, spiinner speed, timeout

        runMovement();

        // pause for servos to move
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void launchBall(double speedServo, double speedSpinner, double timeoutS) {
        robot.s4.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);

        if (opModeIsActive()) {
            elapsedTime.reset();

            robot.spinMotor.setPower(speedSpinner);

            try {
                sleep(1300);
            } catch (Exception e) {
                e.printStackTrace();
            }

            while (opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutS)) {
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

    public void encoderDrive(double speed, double feetDistance, double timeoutS) {
        double leftFeet = feetDistance / 12;
        double rightFeet = feetDistance / 12;
        double leftBackFeet = feetDistance / 12;
        double rightBackFeet = feetDistance / 12;
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if(opModeIsActive()) {
            newLeftTarget = robot.lf.getCurrentPosition() + (int)(leftFeet * COUNTS_PER_INCH);
            newRightTarget = robot.rf.getCurrentPosition() + (int)(rightFeet * COUNTS_PER_INCH);
            newLeftBackTarget = robot.lb.getCurrentPosition() + (int)(leftBackFeet * COUNTS_PER_INCH);
            newRightBackTarget = robot.rb.getCurrentPosition() + (int)(rightBackFeet * COUNTS_PER_INCH);

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
            while (opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutS) && (robot.lf.isBusy() && robot.rf.isBusy())) {

                // Display it for the driver.		                 // Display it for the driver.
                telemetry.addData("Path1",  "Going to %7d :%7d :%7d :%7d", newLeftTarget,  newRightTarget, newLeftBackTarget, newRightBackTarget);		                 telemetry.addData("Path1",  "Going to %7d :%7d :%7d :%7d", newLeftTarget,  newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Currently at %7d :%7d :%7d :%7d",
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
        }
    }

    public abstract void runMovement();
}
