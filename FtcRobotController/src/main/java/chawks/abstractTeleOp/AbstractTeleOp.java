package chawks.abstractTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import chawks.hardware.Dutchess;

import static java.lang.Thread.sleep;

abstract class AbstractTeleOp extends OpMode {

 /*   public final double INCREMENT = 0.01;

    public final double MAX_POS = 1.0;
    public final double MIN_POS = 0.5;
    public double position = (MAX_POS - MIN_POS) / 2;


    public static final double MAX_SPIN_MOTOR_SPEED = 0.03; // lowered from .19 to .1 because ball was shooting to high


    public static final double MAX_SPIN_MOTOR_POWER_DELTA = MAX_SPIN_MOTOR_SPEED / 4;


    public static final double SERVO_SPEED = -1.0;


    public Dutchess robot = new Dutchess();

    public enum DrivingDirection {
        FORWARD, REVERSE;
    }


    public DrivingDirection drivingDirection = DrivingDirection.FORWARD;


    private Thread spinMotorThread;


    public SpinMotorController spinMotorController;


    public class SpinMotorController implements Runnable {

        boolean disabled;


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

        public void incrementUpSpinner() {
            targetPower += .01;
        }

        public void incrementDownSpinner() {
            targetPower -= .01;
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
                    telemetry.addData("Spinner Speed: %.2f", robot.spinMotor.getPower());
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }



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


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        spinMotorController.setTargetPower(MAX_SPIN_MOTOR_SPEED);
        robot.s3.setPower(0.15);
    }



    @Override
    public void loop() {
        handleGamePad1();
        handleGamePad2();
    }


    public abstract void handleGamePad1();


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

    */
}
