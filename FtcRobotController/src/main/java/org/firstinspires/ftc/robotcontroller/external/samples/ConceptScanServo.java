package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Concept: Scan Servo", group = "Concept")
//@Disabled
public class ConceptScanServo extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    public final static double SERVO_HOME = 0.2;
    public final static double SERVO_MIN_RANGE  = 0.20;
    public final static double SERVO_MAX_RANGE  = 0.90;
    // Define class members
    Servo   servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    double servoPosition;


    @Override
   public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        //servo = hardwareMap.servo.get("left_hand");
        servo = hardwareMap.servo.get("s1");
        servoPosition = 0.5;

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

           // servo.setPosition(SERVO_HOME);

            //double C1LX = gamepad1.left_stick_x;
            //double C1LY = -gamepad1.left_stick_x;
            servo.setPosition(SERVO_HOME);

            servo.scaleRange(SERVO_MIN_RANGE, SERVO_MAX_RANGE);
            telemetry.addData("Servo Range Min", "%5.2f", SERVO_MIN_RANGE);
            telemetry.addData("Servo Range Max", "%5.2f", SERVO_MAX_RANGE);

            //servo.setPosition(C1LX);
            //@Override
            //public void loop() {

                if (gamepad1.y) {
                    servoPosition -= SERVO_HOME;
                }
                if (gamepad1.a) {
                    servoPosition += SERVO_HOME;
                }

                // clip the position values so that they never exceed 0..1
                servoPosition = Range.clip(servoPosition, 0, 1);

                // write position values to the servo
                servo.setPosition(servoPosition);
                telemetry.addData("servo", servo.getPosition());
            // slew the servo, according to the rampUp (direction) variable.
           /*
            if (rampUp) {

                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
        */

            // Display the current value
            //telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            servo.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
