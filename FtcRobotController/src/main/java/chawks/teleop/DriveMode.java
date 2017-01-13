package chawks.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import chawks.hardware.DutchessConfiguration;

import static java.lang.Thread.sleep;

@TeleOp(name = "Pushbot: Teleop Tank", group = "Pushbot")
public class DriveMode extends OpMode {
    private final double INCREMENT = 0.01;

    private final double MAX_POS = 1.0;
    private final double MIN_POS = 0.5;
    private double position = (MAX_POS - MIN_POS) / 2;

    public static final double MAX_SPIN_MOTOR_SPEED = 0.38;
    public static final double MAX_SPIN_MOTOR_SPEED_DELTA = MAX_SPIN_MOTOR_SPEED / 4;

    public double servospeed = -1.0;
    public int direction = 1;

    DutchessConfiguration robot = new DutchessConfiguration(); // use the class created to define a Pushbot's hardware
    boolean isReverse;
    boolean isStrafingRight;
    boolean isStrafingLeft;
    boolean wasPad1ButtonY;
    boolean bBttnLstLoop;
    boolean xBttnLstLoop;
    boolean SleepingRoommate;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Let's Go C-HAWKS :D! ");

        setWheelsToRunWithoutEncoder();
        robot.spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        setSpinMotorSpeed(MAX_SPIN_MOTOR_SPEED);
    }

    private double limitSpinSpeed(double speed) {
        if (SleepingRoommate) {
            return 0;
        }
        if (speed < 0) {
            return 0;
        } else if (speed > MAX_SPIN_MOTOR_SPEED) {
            return MAX_SPIN_MOTOR_SPEED;
        } else {
            return speed;
        }
    }

    private void setSpinMotorSpeed(final double targetSpeed) {
        // make sure motor speed is not out of range
        final double speed = limitSpinSpeed(targetSpeed);

        // we can burn the motors if we change the speed too quickly
        double currentSpeed = limitSpinSpeed(robot.spinMotor.getPower());
        double totalDelta = speed - currentSpeed;

        // let's figure out how many steps it would take to change speed gradually
        int steps = Math.max(1, (int) Math.abs(totalDelta / MAX_SPIN_MOTOR_SPEED_DELTA));
        double increment = totalDelta / steps;
        telemetry.addData("setSpinMotorSpeed", "from:%.2f, to:%.2f, steps:%d, increment:%.2f", currentSpeed, speed, steps, increment);

        for (int i = 0; i < steps; i++) {
            currentSpeed += increment;
            robot.spinMotor.setPower(limitSpinSpeed(currentSpeed));
            try {
                // give motor time to adjust
                sleep(250);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,
        // the motor spins faster / more quickly.
        float pad1LeftY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        float pad1RightY = Range.clip(-gamepad1.right_stick_y, -1, 1);
        boolean isPad1ButtonY = gamepad1.y;

        boolean isPad2ButtonA = gamepad2.a;
        boolean isPad2ButtonB = gamepad2.b;
        boolean isPad2DirectionUp = gamepad2.dpad_up;
        boolean isPad2DirectionDown = gamepad2.dpad_down;

        // press Y button to toggle drive direction
        if (false && isPad1ButtonY) {
            if (!wasPad1ButtonY) {
                isReverse = !isReverse;
            }
        }
        wasPad1ButtonY = isPad1ButtonY;

        if (isReverse) {
            // TODO: this logic needs to be reviewed with live testing (Andrew/Joe)
            float temp = -pad1LeftY;
            pad1LeftY = -pad1RightY;
            pad1RightY = temp;
        }

        //telemetry.addData("strafeRight", "RightPow: %.2f" + " , Left: " + "%.2f", leftY, rightY);
        robot.lb.setPower(pad1LeftY);
        robot.lf.setPower(pad1LeftY);
        robot.rf.setPower(pad1RightY);
        robot.rb.setPower(pad1RightY);

        // ---------------------------------------------------------------------------------
        // All of this code is related to Pad 2 handling, which is working fine

        if (isPad2ButtonB) {
            //collector ball
            robot.s4.setPower(servospeed);
            robot.s3.setPower(servospeed);
            robot.s2.setPower(servospeed);

            robot.s4.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (isPad2ButtonA) {
            // Launch ball and move all 3 servos
            robot.s2.setPower(servospeed);
            robot.s3.setPower(servospeed);
            robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (!isPad2ButtonB && !isPad2ButtonA) {
            //back most servo - port 6
            robot.s4.setPower(0);
            // middle servo - port 2
            robot.s3.setPower(0);
            // launch servo - port 4
            robot.s2.setPower(0);
        }

        if (isPad2DirectionUp) {
            position += INCREMENT;
            robot.s1.setPosition(position);
        } else if (isPad2DirectionDown) {
            // this does the same as servoUp goes to same spot - once button is relesed it's dones
            position -= INCREMENT;
            robot.s1.setPosition(position);
        }
    }

    private void setWheelsToRunWithoutEncoder() {
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setWheelsToRunUsingEncoder() {
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetPowerAllWheels() {
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }

    @Override
    public void stop() {
        robot.spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
