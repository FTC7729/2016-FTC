package chawks.hardware;

import com.google.common.collect.Lists;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

public class DutchessConfiguration {
    public static final double MID_SERVO = 0.5;

    public DcMotor lb;
    public DcMotor rb;
    public DcMotor lf;
    public DcMotor rf;
    public DcMotor spinMotor;

    public Servo s1;

    public CRServo s2;
    public CRServo s3;
    public CRServo s4;

    private ElapsedTime period = new ElapsedTime();

    /**
     * Returns a list of motors that we have configured.
     *
     * @return list of motors that we have configured.
     */
    public List<DcMotor> getDcMotors() {
        return Lists.newArrayList(lb, rb, lf, rf, spinMotor);
    }

    public List<DcMotor> getWheels() {
        return Lists.newArrayList(lb, rb, lf, rf);
    }


    public List<CRServo> getCRServos() {
        return Lists.newArrayList(s4, s3, s2);
    }

    /**
     * Initialize the hardware
     * @param hardwareMap configuration from FTC application
     */
    public void init(HardwareMap hardwareMap) {
        // initialize motors
        lb = hardwareMap.dcMotor.get("lB");
        rb = hardwareMap.dcMotor.get("rB");
        lf = hardwareMap.dcMotor.get("lF");
        rf = hardwareMap.dcMotor.get("rF");
        spinMotor = hardwareMap.dcMotor.get("spin");

        // Set all motors to zero power
        resetPowerAllMotors();

        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // left motors go forward
        // note: Set to REVERSE if using AndyMark motors
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);

        // right motors go reverse
        // note: Set to FORWARD if using AndyMark motors
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setWheelsToRunUsingEncoder();

        s1 = hardwareMap.servo.get("s1"); //beacon arm - port 3
        s1.setPosition(0.3);

        s4 = hardwareMap.crservo.get("s4"); //back most servo - port 6
        s3 = hardwareMap.crservo.get("s3"); // middle servo - port 2
        s2 = hardwareMap.crservo.get("s2"); // launch servo - port 4
        resetPowerAllServos();
    }

    private void setWheelsToRunUsingEncoder() {
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetPowerAllServos() {
        for(CRServo servo: getCRServos()) {
            servo.setPower(0);
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        // TODO: s3 has it's own "reset" setting for 0 that we need to figure out!
    }

    public void resetPowerAllMotors() {
        for(DcMotor motor : getDcMotors()) {
            motor.setPower(0);
        }
    }


    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

