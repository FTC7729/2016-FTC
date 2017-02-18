package chawks.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import chawks.hardware.MovementController;

@TeleOp(name = "Test MovementController", group = "Test")
public class TestMovementControllerTeleOp extends AbstractTeleOpMode {
    /**
     * Manages movement of the robot
     */
    private MovementController movementController;

    /**
     * Movement controller runs as a background thread
     */
    private Thread movementThread;

    /**
     * Distance that we want to move robot
     */
    private double distance = 1.0d;

    /**
     * Last time that a button was pressed
     */
    private long lastDpad;

    @Override
    public void start() {
        super.start();

        // initialize movement controller
        movementController = new MovementController(robot, telemetry);
        movementThread = new Thread(movementController);
        movementThread.start();
    }

    @Override
    public void stop() {
        super.stop();
        movementController.stop();
    }

    @Override
    public void handleGamePad1(Gamepad gamepad) {
        long now = System.currentTimeMillis();
        if (gamepad.x) {
            if (now > lastDpad + 1000) {
                movementController.setWheelPower(1d);
                movementController.strafeLeft(distance);
                lastDpad = now;
            }
        } else if (gamepad.y) {
            if (now > lastDpad + 1000) {
                movementController.setWheelPower(1d);
                movementController.strafeRight(distance);
                lastDpad = now;
            }
        } else if (gamepad.a) {
            movementController.setWheelPower(0.4);
            movementController.turn(90);
        } else if (gamepad.b) {
            movementController.setWheelPower(0.4);
            movementController.turn(-90);
        } else if (gamepad.dpad_up) {
            if (now > lastDpad + 100) {
                distance = Range.clip(distance + 0.05, 0, 3);
                lastDpad = now;
            }
        } else if (gamepad.dpad_down) {
            if (now > lastDpad + 100) {
                distance = Range.clip(distance - 0.05, 0, 3);
                lastDpad = now;
            }
        } else if(gamepad.dpad_left || gamepad.dpad_right)  {
            movementController.stopMoving();
        }
        telemetry.addData("teleop", "distance:%.2f", distance);
    }

    @Override
    public void handleGamePad2(Gamepad gamepad) {
    }
}
