package chawks.abstractTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name = "TEST: Teleop Mecanum", group = "Pushbot")
public class AbstractMecanumTeleOp extends AbstractTeleOp {

    @Override
    public void handleGamePad1() {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further
        final boolean isButtonX = gamepad1.x;
        final boolean isButtonY = gamepad1.y;

        // switch driving directions
        if (isButtonX) {
            drivingDirection = DrivingDirection.FORWARD;
        } else if (isButtonY) {
            drivingDirection = DrivingDirection.REVERSE;
        }
        int directionControl = -1;
        final float leftPower;
        final float rightPower;
        switch (drivingDirection) {
            case FORWARD:
            default:
                directionControl = -1;
                break;
            case REVERSE:
                directionControl = 1;
                break;
        }

        float forward = directionControl * gamepad1.left_stick_y; // push joystick1 forward to go forward
        float right = -directionControl * gamepad1.left_stick_x; // push joystick1 to the right to strafe right
        float clockwise = -directionControl * gamepad1.right_stick_x; // push joystick2 to the right to rotate clockwise
        telemetry.addData("pad1", "forward:%.2f, right:%.2f, dir:%s", forward, right, drivingDirection.name());

        final float K = 0.5F; //TODO: DO NOT EXCEED 1
        clockwise = K*clockwise;

        float lf_pow = forward + clockwise + right;
        float rf_pow = forward - clockwise - right;
        float lb_pow = forward + clockwise - right;
        float rb_pow = forward - clockwise + right;

        float max = Math.abs(lf_pow);
        if (Math.abs(rf_pow)>max) max = Math.abs(rf_pow);
        if (Math.abs(lb_pow)>max) max = Math.abs(lb_pow);
        if (Math.abs(rb_pow)>max) max = Math.abs(rb_pow);
        if (max>1) {lf_pow/=max; rf_pow/=max; lb_pow/=max; rb_pow/=max;}

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
