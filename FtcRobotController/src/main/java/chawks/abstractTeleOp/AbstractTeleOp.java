package chawks.abstractTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import chawks.hardware.Dutchess;

import static java.lang.Thread.sleep;

abstract class AbstractTeleOp extends OpMode {
    public final double INCREMENT = 0.01;

    public final double MAX_POS = 1.0;
    public final double MIN_POS = 0.5;
    public double position = (MAX_POS - MIN_POS) / 2;

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
    public Dutchess robot = new Dutchess();

    public enum DrivingDirection {
        FORWARD, REVERSE;
    }

    /**
     * Driving direction
     **/
    public DrivingDirection drivingDirection = DrivingDirection.FORWARD;

    /**
     * Spin motor thread
     **/
    private Thread spinMotorThread;

    /**
     * Spin motor controller
     */
    public SpinMotorController spinMotorController;


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
        robot.s3.setPower(0.15);
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
     * When creating a new Tele-Op user must extend this class then add in custom gamepad1 code
     */
    public abstract void handleGamePad1();

    /**
     * Handle all Game Pad 2 controller input
     * When creating a new Tele-Op user must extend this class then add in custom gamepad2 code
     */
    public abstract void handleGamePad2();

    private void setWheelsToRunWithoutEncoder() {
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setWheelsToRunWithEncoder() {
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
