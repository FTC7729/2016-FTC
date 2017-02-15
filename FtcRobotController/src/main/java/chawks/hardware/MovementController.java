package chawks.hardware;

import com.google.common.base.Preconditions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Handles movement of robot using "dead reckoning", e.g. moving robot for specified distance.
 */
public class MovementController implements Runnable {
    /**
     * Robot that we are controlling
     */
    private final Dutchess robot;

    /**
     * Telemetry used for debugging
     */
    private final Telemetry telemetry;

    /**
     * Power applied to wheels when moving in a forward direction. Should be a positive value.
     */
    private double wheelPower;

    /**
     * If true, we need to exit
     */
    private boolean stopped;

    /**
     * If true, we have initiated a move
     */
    private boolean moving;

    /**
     * Driving direction
     **/
    private DrivingDirection drivingDirection = DrivingDirection.FORWARD;

    /**
     * Time our thread should stop motion at
     */
    private long stopTime;

    public MovementController(Dutchess robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
        setWheelPower(1d); // default
    }

    public double getWheelPower() {
        return wheelPower;
    }

    public void setWheelPower(double wheelPower) {
        this.wheelPower = Range.clip(Math.abs(wheelPower), 0, 1);
        Preconditions.checkState(this.wheelPower > 0, "wheelPower must be positive value greater than 0");
    }

    private boolean isWheelsInPosition() {
        boolean inPosition = true;
        for (DcMotor wheel : robot.getWheels()) {
            if (wheel.isBusy()) {
                telemetry.addData(robot.getNameOfWheel(wheel), "%7d to %7d", wheel.getCurrentPosition(), wheel.getTargetPosition());
                inPosition = false;
            }
        }
        telemetry.update();
        return inPosition;
    }

    public void run() {
        robot.setWheelsToRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setWheelsToRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!stopped) {
            if (moving) {
                if(isWheelsInPosition()) {
                    stopMoving();
                }
            }
        }
        robot.stopAllWheels();
    }

    public DrivingDirection getDrivingDirection() {
        return drivingDirection;
    }

    public void setDrivingDirection(DrivingDirection drivingDirection) {
        this.drivingDirection = drivingDirection;
    }

    public void stop() {
        stopped = true;
    }

    public void stopMoving() {
        moving = false;
        telemetry.addLine("stopped moving!");
        robot.stopAllWheels();
    }

    public void turn(double distanceInches, boolean rightTurn) {
        int sign = rightTurn ? 1 : -1;
        double leftDistanceInches = distanceInches * sign;
        double rightDistanceInches = distanceInches * -sign;
        move(leftDistanceInches, -rightDistanceInches, leftDistanceInches, -rightDistanceInches);
    }

    /**
     * Turn the robot the specified number of degrees.
     *
     * @param angleDegrees The number of degrees to turn. A positive number turns the robot to the right;
     *                     a negative value turns the robot to the left.
     */
    public void turn(double angleDegrees) {
        double angleRadians = angleDegrees * Math.PI / 180;

        // when making a right-hand turn, we reverse the sign of the right wheel distances
        // to emulate tank driving style
        WheelConfiguration config = robot.getWheelConfiguration();
        double leftFrontDistanceInches = angleRadians * config.getRadiusLeftFront();
        double rightFrontDistanceInches = -angleRadians * config.getRadiusRightFront();
        double leftBackDistanceInches = angleRadians * config.getRadiusLeftBack();
        double rightBackDistanceInches = -angleRadians * config.getRadiusRightBack();

        move(leftFrontDistanceInches, rightFrontDistanceInches, leftBackDistanceInches, rightBackDistanceInches);
    }

    public void strafeLeft(double distanceInches) {
        final double leftDistanceInches;
        final double rightDistanceInches;
        switch (drivingDirection) {
            case FORWARD:
            default:
                leftDistanceInches = -distanceInches;
                rightDistanceInches = distanceInches;
                break;
            case REVERSE:
                leftDistanceInches = distanceInches;
                rightDistanceInches = -distanceInches;
                break;
        }
        move(leftDistanceInches, rightDistanceInches, leftDistanceInches, rightDistanceInches);
    }

    public void strafeRight(double distanceInches) {
        final double leftDistanceInches;
        final double rightDistanceInches;
        switch (drivingDirection) {
            case FORWARD:
            default:
                leftDistanceInches = distanceInches;
                rightDistanceInches = -distanceInches;
                break;
            case REVERSE:
                leftDistanceInches = -distanceInches;
                rightDistanceInches = distanceInches;
                break;
        }
        move(leftDistanceInches, rightDistanceInches, leftDistanceInches, rightDistanceInches);
    }

    public void move(double distanceInches) {
        move(distanceInches, distanceInches, distanceInches, distanceInches);
    }

    public void move(double leftDistanceInches, double rightDistanceInches) {
        move(leftDistanceInches, rightDistanceInches, leftDistanceInches, rightDistanceInches);
    }

    /**
     * Move the robot based upon encoder counts. Caller specifies the distances to be moved in inches,
     * and the speed with which the wheels are supposed to move.
     *
     * @param leftFrontDistanceInches  distance to move left-front wheel, in inches (may be negative!)
     * @param rightFrontDistanceInches distance to move right-front wheel, in inches (may be negative!)
     * @param leftBackDistanceInches   distance to move left-back wheel, in inches (may be negative!)
     * @param rightBackDistanceInches  distance to move right-back wheel, in inches (may be negative!)
     */
    private void move(double leftFrontDistanceInches, double rightFrontDistanceInches, double leftBackDistanceInches, double rightBackDistanceInches) {
        if (stopped) {
            // ignore any attempt to move once stopped
            return;
        }

        // convert the distance that we want to travel into encoder "counts"
        final double countsPerInch = robot.getWheelConfiguration().getCountsPerInch();

        robot.setPowerAllWheels(0);
        robot.setWheelsToRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // tell the encoder that we want it move it's "counter" reaches the given target, after which, if our wheel configuration
        // is correct, the robot will have travelled the specified distance
        final int leftTarget = robot.lf.getCurrentPosition() + (int) (leftFrontDistanceInches * countsPerInch);
        final int rightTarget = robot.rf.getCurrentPosition() + (int) (rightFrontDistanceInches * countsPerInch);
        final int leftBackTarget = robot.lb.getCurrentPosition() + (int) (leftBackDistanceInches * countsPerInch);
        final int rightBackTarget = robot.rb.getCurrentPosition() + (int) (rightBackDistanceInches * countsPerInch);

        RobotLog.i("move: %.2f, %.2f, %.2f, %.2f", leftFrontDistanceInches, rightFrontDistanceInches, leftBackDistanceInches, rightBackDistanceInches);
        RobotLog.i("from: %d, %d, %d, %d", robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition());
        RobotLog.i("to:   %d, %d, %d, %d", leftTarget, rightTarget, leftBackTarget, rightBackTarget);

        // tell robot wheels where to go
        moving = true;
        robot.lf.setTargetPosition(leftTarget);
        robot.rf.setTargetPosition(rightTarget);
        robot.lb.setTargetPosition(leftBackTarget);
        robot.rb.setTargetPosition(rightBackTarget);
        robot.setPowerAllWheels(wheelPower);
    }

    @Override
    public String toString() {
        return "MovementController{" +
                "wheelPower=" + wheelPower +
                '}';
    }
}
