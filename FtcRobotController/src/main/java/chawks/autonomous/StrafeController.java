package chawks.autonomous;

import chawks.hardware.Dutchess;

/**
 * Created by joseph on 2/4/17.
 */
public class StrafeController implements Runnable {
    public static enum StrafeDirection {
        LEFT, RIGHT, STANDSTILL
    }

    /**
     * Robot that we are controlling
     */
    private final Dutchess robot;

    /**
     * Speed at which we are moving, must be -1 to 1, inclusive
     */
    private float strafeSpeed;

    /**
     * Direction that we are moving
     */
    private StrafeDirection direction;

    /**
     * If true, we need to exit
     */
    private boolean stopped;

    /**
     * If true, we do not move the robot
     */
    private boolean enabled;


    public StrafeController(Dutchess robot, float strafeSpeed) {
        this.robot = robot;
        this.direction = StrafeDirection.STANDSTILL;
        setStrafeSpeed(strafeSpeed);
    }

    public float getStrafeSpeed() {
        return strafeSpeed;
    }

    public void setStrafeSpeed(float strafeSpeed) {
        this.strafeSpeed = Math.min(Math.max(-1.0f,strafeSpeed),1.0f);
    }

    public StrafeDirection getDirection() {
        return direction;
    }

    public void setDirection(StrafeDirection direction) {
        this.direction = direction;
    }

    public void run() {
        while (!stopped) {
            if (enabled) {
                switch (direction) {
                    case STANDSTILL:
                        resetPowerAllWheels();
                        break;
                    case LEFT:
                        strafeLeft();
                        break;
                    case RIGHT:
                        strafeRight();
                        break;
                }
            }
        }
    }

    public void stop() {
        stopped = true;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void strafeLeft() {
        float tempLeft;
        float tempRight;
        tempLeft = strafeSpeed;
        tempRight = -strafeSpeed;
        robot.lf.setPower(tempLeft);
        robot.rf.setPower(tempRight);
        robot.lb.setPower(-tempLeft);
        robot.rb.setPower(-tempRight);
    }

    public void strafeRight() {
        float tempLeft;
        float tempRight;
        tempLeft = strafeSpeed;
        tempRight = -strafeSpeed;
        robot.lf.setPower(tempLeft);
        robot.rf.setPower(tempRight);
        robot.lb.setPower(-tempLeft);
        robot.rb.setPower(-tempRight);
    }

    @Override
    public String toString() {
        return "StrafeController{" +
                "strafeSpeed=" + strafeSpeed +
                ", direction=" + direction +
                '}';
    }

    private void resetPowerAllWheels() {
        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }
}
