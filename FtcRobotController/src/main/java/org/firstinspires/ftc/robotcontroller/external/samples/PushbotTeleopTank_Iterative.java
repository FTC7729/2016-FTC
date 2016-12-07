/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.opmode.TestableVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.Color;
import org.lasarobotics.vision.util.color.ColorGRAY;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

import static android.R.attr.scaleX;
import static android.R.attr.x;
import static android.R.attr.y;
import static com.qualcomm.robotcore.util.Range.scale;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
@Disabled
public class PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
  // double          clawOffset  = 0.0 ;                  // Servo mid position
   // final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    final static double CLAW_MIN_RANGE = 0.20;
    final static double CLAW_MAX_RANGE = 0.7;
    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARDS_SPEED    = -0.6;
    public double           spinnerSpeed = 0.20;
    public double           spinnerSpeedBack = -spinnerSpeed;
    public double           spinnerSpeedBack2 = -0.40;
    public double           spinnerSpeedBack3 = -0.60;
    public double           spinnerSpeedBack4 = -0.80;
    private double          scoopUp;
    // position of the claw servo
    double clawPosition;
    // amount to change the claw servo position by
    double clawDelta = 0.1;

    //double servroMe;
    public final static double SERVO_HOME = 0.2;
    public final static double SERVO_MIN_RANGE  = 0.20;
    public final static double SERVO_MAX_RANGE  = 0.90;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver :D! ");

        clawPosition = 0.2;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        double left;
        double right;
        */
        //
        /* Set the Gamepad values */
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        boolean A1isPressed = gamepad1.a;
        boolean X1isPressed = gamepad1.x;
        boolean Y1isPressed = gamepad1.y;
        boolean B1isPressed = gamepad1.b;

        boolean A2isPressed = gamepad2.a;
        boolean X2isPressed = gamepad2.x;
        boolean Y2isPressed = gamepad2.y;
        boolean B2isPressed = gamepad2.b;

        final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        final int    CYCLE_MS    =   50;     // period of each cycle
        final double MAX_POS     =  1.0;     // Maximum rotational position
        final double MIN_POS     =  0.0;     // Minimum rotational position

        // Define class members

        double  position = MIN_POS; //= (MAX_POS - MIN_POS) / 2; // Start at halfway position
        boolean rampUp = true;

        /*
        motorRight.setPower(right);
        motorLeft.setPower(left);
        Before this you have to make sure to “clip” the joystick values to they never go above 1 and below -1, because those are the only value range that the motors now take.  To do this:
        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
         */
        // we know the left_stick_x works
        //If we want the Spin Wheel to be controlled manually with the Joy-Stick
            //enable the lines of code below:
                //scoopUp = -gamepad1.left_stick_x;
                //robot.spinMotor.setPower(scoopUp);

        if (gamepad1.dpad_right){

            strafeRight();


        }

       else if (gamepad1.dpad_left){

            strafeLeft();


        }
        else if (B1isPressed) {

           // spinSet();

        }

       else if (X1isPressed) {

            // Keep stepping up until we hit the max value.
           // position += INCREMENT ;
            robot.servo1.setDirection(Servo.Direction.FORWARD);
           // if (position >= MAX_POS ) {
           //     position = MAX_POS;
                //rampUp = !rampUp;   // Switch ramp direction
          //  }
         //   robot.servo1.setPosition(position);
         //   telemetry.addData("Robot Status", "Shooting ball Stop"+ String.format("%.2f", clawPosition));
         //   robot.spinMotor.setPower(0);

        }

        else if (Y1isPressed) {

            robot.servo1.setDirection(Servo.Direction.REVERSE);

            // Keep stepping up until we hit the max value.
            //position -= INCREMENT ;
            //if (position <= MIN_POS ) {
           //     position = MIN_POS;
                //rampUp = !rampUp;  // Switch ramp direction
           // }
            robot.servo1.setPosition(position);
            //   telemetry.addData("Robot Status", "Shooting ball Stop"+ String.format("%.2f", clawPosition));
            //   robot.spinMotor.setPower(0);

        }
// Figuring out which speed we need to launch the balls to get into Vortex
        else if (A2isPressed) {

            spinSet();

        }

        else if (X2isPressed) {

            spinSet2();

        }

        else if (Y2isPressed) {

            spinSet3();

        }

        else if (B2isPressed) {

            spinSet4();

        }
//This is from Max to have the robot add the spinner
        /*
        else if (gamepad1.b){
            // notifies user that launcher will shoot ball
            telemetry.addData("Robot Status", "Shooting ball !");


            for( int i=1;  i<3; i++){
               // spinnerSpeed = spinnerSpeed
                robot.spinMotor.setPower(spinnerSpeed);

                try {
                    Thread.sleep(10);
                }catch (InterruptedException e){
                    e.printStackTrace();
                }

                if (spinnerSpeed == 0.6){

                    robot.spinMotor.setPower(0.3);
                    try {
                        Thread.sleep(10);
                    }catch (InterruptedException e){
                        e.printStackTrace();
                    }

                    return;
                }

            }


        } */

        else{
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // This turns the robot around with the left joystick

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            robot.leftMotorBack.setPower(left);
            robot.rightMotorBack.setPower(right);
            //telemetry.addData("Robot Status", "Shooting ball Stop "+ String.format("%.2f", spinnerSpeedBack));
            //If B is not pressed, STOP
            robot.spinMotor.setPower(0);
            robot.servo1.setPosition(0);

            //right = Range.clip(right, -1, 1);
            //left = Range.clip(left, -1, 1);

        }


//hold down for a few seconds it goes wild - fix strafing


        // Use gamepad left & right Bumpers to open and close the claw


        // Move both servos to new position.  Assume servos are mirror image of each other.

        // robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        //  robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)


        // Send telemetry message to signify robot running;

    }

    public void spinSet(){
        //telemetry.addData("Robot Status", "Shooting ball "+ String.format("%.2f", spinnerSpeedBack));
        robot.spinMotor.setPower(spinnerSpeedBack);
        //clawPosition -= clawDelta;
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

    }

    public void spinSet2(){
        //telemetry.addData("Robot Status", "Shooting ball "+ String.format("%.2f", spinnerSpeedBack));
        robot.spinMotor.setPower(spinnerSpeedBack2);
        //clawPosition -= clawDelta;
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

    }
    public void spinSet3(){
        //telemetry.addData("Robot Status", "Shooting ball "+ String.format("%.2f", spinnerSpeedBack));
        robot.spinMotor.setPower(spinnerSpeedBack3);
        //clawPosition -= clawDelta;
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

    }
    public void spinSet4(){
        //telemetry.addData("Robot Status", "Shooting ball "+ String.format("%.2f", spinnerSpeedBack));
        robot.spinMotor.setPower(spinnerSpeedBack4);
        //clawPosition -= clawDelta;
        //clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

    }


    public void strafeRight(){

            /*
            rf: +1
            lf: -1
            rb: -1
            lb: +1

            rf: x
            lf: -x
            rb: -x
            lb: x
             ---- > together we have
                rf: y+x
                lf: y-x
                rb: y-x
                lb: y+x

            ----- > how to do diagonals, turns
            rf: -1
            lf: +1
            rb: -1
            lb: +1

                That's Z axis
                rf: -z
                lf: +z
                rb: -z
                lb: +z
            --->>>> How put x+y+z together
                    rf: y+x-z
                    lf: y-x+z
                    rb: y-x-z
                    lb: y+x+z

             */
        /*
            frontright.setPower(scale(y+x-z));
            frontleft.setPower(scale(y-x+z));
            backright.setPower(scale(y-x-z));
            backleft.setPower(scale(y+x+z));
         */
        /*
        robot.rightMotor.setPower(FORWARD_SPEED);
        robot.leftMotor.setPower(BACKWARDS_SPEED);
        robot.rightMotorBack.setPower(BACKWARDS_SPEED);
        robot.leftMotorBack.setPower(FORWARD_SPEED);
        */

        robot.rightMotor.setPower(FORWARD_SPEED);
        robot.leftMotor.setPower(BACKWARDS_SPEED);
        robot.rightMotorBack.setPower(BACKWARDS_SPEED);
        robot.leftMotorBack.setPower(FORWARD_SPEED);

        telemetry.addData("strafeRight", "%.2f", "%.2f", FORWARD_SPEED, BACKWARDS_SPEED);


    }


    public void strafeLeft(){

        robot.rightMotor.setPower(BACKWARDS_SPEED);
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.rightMotorBack.setPower(FORWARD_SPEED);
        robot.leftMotorBack.setPower(BACKWARDS_SPEED);



        telemetry.addData("strafeLeft", "%.2f", "%.2f", FORWARD_SPEED, BACKWARDS_SPEED);



    }
/*
    public void feedShooter(double power, long time) throws Exception{

        robot.picker.setPower(power);

        try {
            Thread.sleep(time);
        }
        catch (Exception e){

            e.printStackTrace();


        }


    }

*/
    /*
     * Code to run ONCE after the driver hits STOP
     */
    //Assertion failed: stop() should be called only if start() called before
    @Override
    public void stop() {
    }

}
