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
    private double distance;

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
            movementController.strafeLeft(distance);
        } else if (gamepad.y) {
            movementController.strafeRight(distance);
        } else if (gamepad.a) {
            movementController.turn(90);
        } else if (gamepad.b) {
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
        }
        telemetry.addData("teleop", "distance:%.2f", distance);
    }

    @Override
    public void handleGamePad2(Gamepad gamepad) {
    }
}
