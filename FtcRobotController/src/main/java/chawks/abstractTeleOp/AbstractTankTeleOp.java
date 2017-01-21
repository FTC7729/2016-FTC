package chawks.abstractTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TEST: Teleop Tank", group = "Pushbot")
public class AbstractTankTeleOp extends AbstractTeleOp {

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
        if (isButtonX) {
            drivingDirection = DrivingDirection.FORWARD;
        } else if (isButtonY) {
            drivingDirection = DrivingDirection.REVERSE;
        }

        if (isDPADLeft) {
            float tempLeft;
            float tempRight;
            switch (drivingDirection) {
                case FORWARD:
                default:
                    tempLeft = -STRAFE_SPEED;
                    tempRight = STRAFE_SPEED;
                    break;
                case REVERSE:
                    tempLeft = STRAFE_SPEED;
                    tempRight = -STRAFE_SPEED;
                    break;
            }
            robot.lf.setPower(tempLeft);
            robot.rf.setPower(tempRight);
            robot.lb.setPower(-tempLeft);
            robot.rb.setPower(-tempRight);
            return;
        } else if (isDPADRight) {
            float tempLeft;
            float tempRight;
            switch (drivingDirection) {
                case FORWARD:
                default:
                    tempLeft = STRAFE_SPEED;
                    tempRight = -STRAFE_SPEED;
                    break;
                case REVERSE:
                    tempLeft = -STRAFE_SPEED;
                    tempRight = STRAFE_SPEED;
                    break;
            }
            robot.lf.setPower(tempLeft);
            robot.rf.setPower(tempRight);
            robot.lb.setPower(-tempLeft);
            robot.rb.setPower(-tempRight);
            return;
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
