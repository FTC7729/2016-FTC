package chawks.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import chawks.hardware.Dutchess;

/**
 * Tests the spinner which is used for shooting
 *
 * Test procedure:
 * <ul>
 *     <li>Press Init</li>
 *     <li>Press Play</li>
 *     <li>Motor should spin up for 5 seconds</li>
 *     <li>Motor should stop</li>
 * </ul>
 *
 * @author Joseph Arakelian
 */
@TeleOp(name="SpinnerEncoderTest", group="Test")
public class SpinnerEncoderTest extends OpMode {

    private State state;
    private Dutchess robot = new Dutchess();
    private long stopTime;

    public void init() {
        robot.init(hardwareMap);
        state = State.ResetState;
    }

    public void loop() {
        switch (state) {
            case ResetState:
                robot.spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                state = State.SetEncoderPower;
                break;

            case SetEncoderPower:
                double currentPower = robot.spin.getPower();
                if (currentPower < .5) {
                    robot.spin.setPower(Math.min(currentPower + .1, 0.5));
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                } else {
                    state = State.WaitForTime;
                    stopTime = System.currentTimeMillis() + 5000;
                }
                break;

            case WaitForTime:
                long now = System.currentTimeMillis();
                if (now > stopTime) {
                    state = State.SlowMotor;
                }
                break;

            case SlowMotor:
                robot.spin.setPower(0);
                state = State.Done;
                break;
        }
    }

    enum State {
        ResetState, SetEncoderPower, WaitForTime, SlowMotor, Done
    }
}
