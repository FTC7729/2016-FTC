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


    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARDS_SPEED    = -0.6;
    public double              spinnerSpeed = .10;

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
        telemetry.addData("Say", "Hello Driver :D! ");    //
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
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;


        /*
        motorRight.setPower(right);
        motorLeft.setPower(left);
        Before this you have to make sure to “clip” the joystick values to they never go above 1 and below -1, because those are the only value range that the motors now take.  To do this:
        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
         */

        if (gamepad1.dpad_right){

            strafeRight();


        }

       else if (gamepad1.dpad_left){

            strafeLeft();


        }

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



                if (spinnerSpeed == .6){

                    robot.picker.setPower(.3);
                    try {
                        Thread.sleep(10);
                    }catch (InterruptedException e){
                        e.printStackTrace();
                    }

                    return;
                }

            }





        }




        else{
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            // This turns the robot around with the left joystick

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            robot.leftMotorBack.setPower(left);
            robot.rightMotorBack.setPower(right);

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

        telemetry.addData("right", "%.2f", FORWARD_SPEED, BACKWARDS_SPEED);


    }

    public void strafeLeft(){
        robot.leftMotor.setPower(FORWARD_SPEED);
        robot.leftMotorBack.setPower(BACKWARDS_SPEED);
        telemetry.addData("left", "%.2f", FORWARD_SPEED, BACKWARDS_SPEED);
        robot.rightMotorBack.setPower(FORWARD_SPEED);
        robot.rightMotor.setPower(BACKWARDS_SPEED);




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
