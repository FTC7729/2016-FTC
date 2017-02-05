package chawks.abstractTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TEST: Teleop Tank", group = "Pushbot")
public class HolonomicTankTeleOp extends AbstractTeleOp {

    public float STRAFE_SPEED = 0.6F;

    @Override
    public void handleGamePad1() {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,
        float leftStickY = Range.clip(-gamepad1.left_stick_y, -1, 1);
        float rightStickY = Range.clip(-gamepad1.right_stick_y, -1, 1);
        final boolean isButtonX = gamepad1.x;
        final boolean isButtonY = gamepad1.y;
        telemetry.addData("pad1", "left:%.2f, right:%.2f, dir:%s", leftStickY, rightStickY, drivingDirection.name());

        final boolean isDPADLeft = gamepad1.dpad_left;
        final boolean isDPADRight = gamepad1.dpad_right;

        // switch driving directions
        if (isButtonY) {
            if (drivingDirection == DrivingDirection.FORWARD) {
                drivingDirection = DrivingDirection.REVERSE;
            } else if (drivingDirection == DrivingDirection.REVERSE) {
                drivingDirection = DrivingDirection.FORWARD;
            } else {
                drivingDirection = DrivingDirection.FORWARD;
            }
        }


        /**
         * The following switch statement handles the direction (Forward or Backwards
         */
        int directionControl = 1;
        switch (drivingDirection) {
            case FORWARD:
                directionControl = 1;
                break;
            case REVERSE:
                directionControl = -1;
                break;
        }

        /**
         * Set the power of forward, right, and clockwise motion in order to initiate drive system
         */

        /** push both joysticks forward to go forward, push both joysticks backwards to go backwards */
        float forward = -directionControl * ((gamepad1.right_stick_y + gamepad1.left_stick_y) / 2);

        /** push the joystick to the right to strafe right, in combination with the "forward" value the robot will be able to move holistically */
        float right = directionControl * (gamepad1.left_stick_x);

        /** push left forward and right backwards to turn right, and vice versa to turn left :: This type is used to rotate the robot without conflict */
        float clockwise = directionControl * ((gamepad1.right_stick_y - gamepad1.left_stick_y) / 2);

        /**
         * The value "K" should never equal or be less than 0 as there will be no clockwise movement
         *
         * The value of "K" should never be greater than "1" as it will mess with values within the program, making them larger than the motors can handle
         */
        final float K = 0.5F;

        /**
         * We multiply by "K" in order to manipulate the speed of turning
         * If turn speed is too high, go above and change the value of "K" to something lower
         * If turn speed is too low, go above and change the value of "K" to something higher
         */
        clockwise = K*clockwise;

        /**
         * Generate powers required to move each wheel in this type of movement
         * Each wheel must have its own generated type of power because if it didn't there wouldn't be mecanum movement
         */
        float lf_pow = forward + clockwise + right;
        float rf_pow = forward - clockwise - right;
        float lb_pow = forward + clockwise - right;
        float rb_pow = forward - clockwise + right;

        /**
         * Limits the movement powers to a range dependent upon the max
         * If the max exceeds "1", all of the powers are divided by the max in order to make them smaller and in doing such limiting them all to "-1" and "1"
         */
        float max = Math.abs(lf_pow);
        if (Math.abs(rf_pow)>max) max = Math.abs(rf_pow);
        if (Math.abs(lb_pow)>max) max = Math.abs(lb_pow);
        if (Math.abs(rb_pow)>max) max = Math.abs(rb_pow);
        if (max>1) {lf_pow/=max; rf_pow/=max; lb_pow/=max; rb_pow/=max;}

        /**
         * Sets the power of each motor to their respective powers
         */
        robot.lb.setPower(lb_pow);
        robot.rb.setPower(rb_pow);
        robot.rf.setPower(rf_pow);
        robot.lf.setPower(lf_pow);
    }

    @Override
    public void handleGamePad2() {
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

}
