package chawks.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import chawks.hardware.DrivingDirection;
import chawks.hardware.ShootingController;

@TeleOp(name = "Tank", group = "TeleOp")
public class TankTeleOp extends AbstractTeleOpWithSpinner {

    public float STRAFE_SPEED = 0.6F;

    @Override
    public void handleGamePad1(Gamepad gamepad) {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,
        float leftStickY = Range.clip(-gamepad.left_stick_y, -1, 1);
        float rightStickY = Range.clip(-gamepad.right_stick_y, -1, 1);
        final boolean isButtonX = gamepad.x;
        final boolean isButtonY = gamepad.y;
        telemetry.addData("pad1", "left:%.2f, right:%.2f, dir:%s", leftStickY, rightStickY, drivingDirection.name());

        final boolean isDPADLeft = gamepad.dpad_left;
        final boolean isDPADRight = gamepad.dpad_right;

        // switch driving directions
        if (isButtonX) {
            drivingDirection = DrivingDirection.FORWARD;
        } else if (isButtonY) {
            drivingDirection = DrivingDirection.REVERSE;
        }

        // if either the DPAD left/right buttons are depressed, then we are strafing and setting the
        // wheel power to move left or right
        if (isDPADLeft) {
            float leftPower;
            float rightPower;
            switch (drivingDirection) {
                case FORWARD:
                default:
                    leftPower = -STRAFE_SPEED;
                    rightPower = STRAFE_SPEED;
                    break;
                case REVERSE:
                    leftPower = STRAFE_SPEED;
                    rightPower = -STRAFE_SPEED;
                    break;
            }
            robot.lf.setPower(leftPower);
            robot.rf.setPower(rightPower);
            robot.lb.setPower(-leftPower);
            robot.rb.setPower(-rightPower);
            return;
        } else if (isDPADRight) {
            float leftPower;
            float rightPower;
            switch (drivingDirection) {
                case FORWARD:
                default:
                    leftPower = STRAFE_SPEED;
                    rightPower = -STRAFE_SPEED;
                    break;
                case REVERSE:
                    leftPower = -STRAFE_SPEED;
                    rightPower = STRAFE_SPEED;
                    break;
            }
            robot.lf.setPower(leftPower);
            robot.rf.setPower(rightPower);
            robot.lb.setPower(-leftPower);
            robot.rb.setPower(-rightPower);
            return;
        }

        // the moment we take our finger off the DPAD, we are using the left and right stick values
        // to determine the power to apply to wheels.
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
    public void handleGamePad2(Gamepad gamepad) {
        boolean isButtonA = gamepad.a;
        boolean isButtonB = gamepad.b;
        boolean isDirectionUp = gamepad.dpad_up;
        boolean isDirectionDown = gamepad.dpad_down;
        telemetry.addData("pad2", "a:%s, b:%s, up:%s, down:%s, lb:%s, rb%s",
                isButtonA, isButtonB, isDirectionUp, isDirectionDown, gamepad.left_bumper, gamepad.right_bumper);

        if (gamepad.left_bumper) {
            shootingController.setTargetPower(0);
        } else if (gamepad.right_bumper) {
            shootingController.setTargetPower(ShootingController.MAX_SPIN_MOTOR_SPEED);
        }

        if (isButtonB) {
            // collect a ball
            double armSpeed = armController.getArmSpeed();
            robot.s4.setPower(armSpeed);
            robot.s3.setPower(armSpeed);
            robot.s2.setPower(armSpeed);

            robot.s4.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s3.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.s2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (isButtonA) {
            // launch ball and move all 3 servos
            double armSpeed = armController.getArmSpeed();
            robot.s2.setPower(armSpeed);
            robot.s3.setPower(armSpeed);
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
            armController.adjustPosition(armController.getIncrement());
        } else if (isDirectionDown) {
            armController.adjustPosition(-armController.getIncrement());
        }
    }

}