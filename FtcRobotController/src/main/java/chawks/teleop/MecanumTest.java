package chawks.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import chawks.hardware.Dutchess;

import static java.lang.Thread.sleep;

@TeleOp(name = "Test: Teleop Mecanum", group = "Pushbot")
public class MecanumTest extends OpMode {

    Dutchess robot = new Dutchess();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Let's Go C-HAWKS :D! ");

        setWheelsToRunWithoutEncoder();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    int directionControl = -1;

    @Override
    public void loop() {
        float forward = directionControl * gamepad1.left_stick_y; // push joystick1 forward to go forward
        float right = -directionControl * gamepad1.left_stick_x; // push joystick1 to the right to strafe right
        float clockwise = gamepad1.right_stick_x; // push joystick2 to the right to rotate clockwise

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
    public void stop() {}
}
