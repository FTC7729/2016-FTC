package chawks.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import chawks.hardware.Dutchess;

import static java.lang.Thread.sleep;

//@TeleOp(name = "Comp: Teleop Tank", group = "Pushbot")
public class DriveMode extends OpMode {
    private final double INCREMENT = 0.01;

    private final double MAX_POS = 1.0;
    private final double MIN_POS = 0.5;
    private double position = (MAX_POS - MIN_POS) / 2;

    /**
     * Maximum speed of motor
     **/
    public static final double MAX_SPIN_MOTOR_SPEED = 0.15;

    /**
     * Maximum amount we are willing to change motor speed at-a-time
     **/
    public static final double MAX_SPIN_MOTOR_POWER_DELTA = MAX_SPIN_MOTOR_SPEED / 4;

    /**
     * Servo speed for shooting
     */
    public static final double SERVO_SPEED = -1.0;

    /**
     * Robot hardware configuration
     */
    private Dutchess robot = new Dutchess();

    public enum DrivingDirection {
        FORWARD, REVERSE;
    }

    /**
     * Driving direction
     **/
    private DrivingDirection drivingDirection = DrivingDirection.FORWARD;

    /**
     * Spin motor thread
     **/
    private Thread spinMotorThread;

    /**
     * Spin motor controller
     */
    private SpinMotorController spinMotorController;


    public class SpinMotorController implements Runnable {
        /**
         * True if spin motor should be disabled
         **/
        boolean disabled;

        /**
         * Target spin motor speed. This background thread will continously work to reach
         * this speed, at a safe pace.
         */
        private double targetPower;

        public SpinMotorController(boolean disabled, double power) {
            this.disabled = disabled;
            setTargetPower(power);
        }

        public void setTargetPower(double power) {
            this.targetPower = disabled ? 0 : clipSpinMotorPower(power);
        }

        public void disable() {
            telemetry.addData("spinMotor", "currentPower:%s", robot.spinMotor.getPower());
            setTargetPower(0);
            this.disabled = true;
        }

        private double clipSpinMotorPower(double power) {
            if (power < 0) {
                return 0;
            } else if (power > MAX_SPIN_MOTOR_SPEED) {
                return MAX_SPIN_MOTOR_SPEED;
            } else {
                return power;
            }
        }

        public void run() {
            telemetry.addData("spinMotor", "started");
            try {
                managePower();
            } finally {
                telemetry.addData("spinMotor", "stopped");
            }
        }

        private void managePower() {
            while (true) {
                // make sure spin motor is going expected direction
                DcMotorSimple.Direction direction = robot.spinMotor.getDirection();
                if (direction != DcMotorSimple.Direction.FORWARD) {
                    telemetry.addData("spinMotor", "directionWas:%s", direction.name());
                    robot.spinMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                // we can burn the motors if we change the speed too quickly
                final double currentPower = clipSpinMotorPower(robot.spinMotor.getPower());

                // figure out what new power settings can be for this iteration
                final double newPower;
                double delta = targetPower - currentPower;
                if (delta > 0) {
                    newPower = clipSpinMotorPower(currentPower + Math.min(delta, MAX_SPIN_MOTOR_POWER_DELTA));
                } else {
                    newPower = clipSpinMotorPower(currentPower - Math.min(-delta, MAX_SPIN_MOTOR_POWER_DELTA));
                }

                telemetry.addData("spinMotor", "from:%.2f, to:%.2f", currentPower, newPower);

                robot.spinMotor.setPower(newPower);
                if (newPower <= 0.0 && disabled) {
                    // exit thread when disabled and target power reached
                    return;
                }

                try {
                    // give motor time to adjust
                    sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }


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

        boolean sleepingRoommate = false;
        spinMotorController = new SpinMotorController(sleepingRoommate, 0);
        spinMotorThread = new Thread(spinMotorController);
        spinMotorThread.start();
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
        spinMotorController.setTargetPower(MAX_SPIN_MOTOR_SPEED);
        robot.s3.setPower(-0.15);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        handleGamePad1();
        handleGamePad2();
    }

    /**
     * Handle all Game Pad 1 controller input
     */
    private void handleGamePad1() {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,
        float leftStickY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        float rightStickY = Range.clip(-gamepad1.right_stick_y, -1, 1);
        final boolean isButtonX = gamepad1.x;
        final boolean isButtonY = gamepad1.y;
        telemetry.addData("pad1", "left:%.2f, right:%.2f, dir:%s", leftStickY, rightStickY, drivingDirection.name());

        // switch driving directions
        if (isButtonX) {
            drivingDirection = DrivingDirection.FORWARD;
        } else if (isButtonY) {
            drivingDirection = DrivingDirection.REVERSE;
        }

        final float leftPower;
        final float rightPower;
        switch (drivingDirection) {
            case FORWARD:
            default:
                leftPower = leftStickY;
                rightPower = rightStickY;
                break;
            case REVERSE:
                leftPower = -rightStickY;
                rightPower = -leftStickY;
                break;
        }

        robot.lb.setPower(leftPower);
        robot.lf.setPower(leftPower);
        robot.rf.setPower(rightPower);
        robot.rb.setPower(rightPower);
    }

    /**
     * Handle all Game Pad 2 controller input
     */
    private void handleGamePad2() {
        boolean isButtonA = gamepad2.a;
        boolean isButtonB = gamepad2.b;
        boolean isDirectionUp = gamepad2.dpad_up;
        boolean isDirectionDown = gamepad2.dpad_down;
        telemetry.addData("pad2", "a:%s, b:%s, up:%s, down:%s, lb:%s, rb%s",
                isButtonA, isButtonB, isDirectionUp, isDirectionDown, gamepad2.left_bumper, gamepad2.right_bumper);

        if (gamepad2.left_bumper) {
            spinMotorController.setTargetPower(0);
        } else if (gamepad2.right_bumper) {
            spinMotorController.setTargetPower(MAX_SPIN_MOTOR_SPEED);
        }

        if (isButtonB) {
            //collector ball
            robot.s4.setPower(SERVO_SPEED);
            robot.s3.setPower(SERVO_SPEED);
            robot.s2.setPower(SERVO_SPEED);

            robot.s4.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (isButtonA) {
            // Launch ball and move all 3 servos
            robot.s2.setPower(SERVO_SPEED);
            robot.s3.setPower(SERVO_SPEED);
            robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (!isButtonB && !isButtonA) {
            //back most servo - port 6
            robot.s4.setPower(0);
            // middle servo - port 2
            robot.s3.setPower(0);
            // launch servo - port 4
            robot.s2.setPower(0);
        }

        if (isDirectionUp) {
            position += INCREMENT;
            robot.s1.setPosition(position);
        } else if (isDirectionDown) {
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
        spinMotorController.disable();
    }
}
