package chawks.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import chawks.hardware.Dutchess;

@Autonomous(name = "Shoot from corner", group = "Pushbot")
public class ShootFromCorner extends LinearOpMode {
    /**
     * Robot configuration
     **/
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

        robot.spinMotor.setPower(0.4);
        sleep(250);

        launchBall(-1.0, 0.8, 5.0); //servo speed, spiinner speed, timeout

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
}
