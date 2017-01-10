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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * make
 * shit
 * drive
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Pushbot: Teleop Tank", group = "Pushbot")
@Disabled
public class PushbotTeleopTank_Iterative extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    final boolean FALSE = false;
    final boolean TRUE = true;
    boolean isReverse = FALSE;
    boolean isStrafing = FALSE;
    private static final double EXPO = 1.3;
    public double spinnerSpeedBack5 = 0.40;
    boolean yBttnLstLoop = FALSE;
    boolean bBttnLstLoop = FALSE;
    final double INCREMENT = 0.01;
    final double MAX_POS = 1.0;     // Maximum rotational position
    final double MIN_POS = 0.0;     // Minimum rotational position
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public int direction = 1;


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
        telemetry.addData("Say", "Let's Go C-HAWKS :D! ");

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

        robot.servo1.setPosition(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        double left;
        double rightY;
        */
        //
        /* Set the Gamepad values */
        float leftY = (float) -Math.pow(gamepad1.left_stick_y, EXPO);
        float rightY = (float) -Math.pow(gamepad1.right_stick_y, EXPO);

        boolean A2isPressed = gamepad2.a;

        boolean B2isPressed = gamepad2.b;
        boolean ServoBeaconUp = gamepad2.dpad_up;
        boolean ServoBeaconDown = gamepad2.dpad_down;

        float LeftLoadBall = gamepad2.left_trigger; // collect ball, first phase, s2 & s3
        float RightFireBall = gamepad2.right_trigger; // collect ball, second phase, s3 & s4


        // Use B button to toggle direction of robot
        if (gamepad1.b) {
            if (!bBttnLstLoop) {
                bBttnLstLoop = TRUE;
                isReverse = !isReverse;
            }
        } else {
            bBttnLstLoop = FALSE;
        }

        // Use Y button to toggle strafe mode of robot
        if (gamepad1.y) {
            if (!yBttnLstLoop) {
                yBttnLstLoop = TRUE;
                isStrafing = !isStrafing;
                if (isStrafing) {
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else {
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        } else {
            yBttnLstLoop = FALSE;
        }

        if (isStrafing) {

        } else {
            // revese the controls if we are in reverse mode
            if (isReverse) {
                leftY = -leftY;
                rightY = -rightY;
                float temp = leftY;
                leftY = rightY;
                rightY = temp;
            }


            robot.leftMotorBack.setPower(leftY);
            robot.leftMotor.setPower(leftY);
            robot.rightMotor.setPower(rightY);
            robot.rightMotorBack.setPower(rightY);

        }


        // Define class members


        //setting servoUp and down - does the max only not by increments and pausse

        //we want the spinner to constantly be moing for the whole opmode
        robot.spinMotor.setPower(spinnerSpeedBack5);

        if (gamepad1.start) {
            direction = -direction;
        }

        if (B2isPressed) {

            robot.crservo2.setPower(-0.5);
            robot.crservo3.setPower(-0.5);

            robot.crservo2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.crservo3.setDirection(DcMotorSimple.Direction.REVERSE);


        }

        if (A2isPressed) {
            // Launch ball and move all 3 servos

            robot.crservo2.setPower(-0.5);
            robot.crservo4.setPower(-0.5);
            robot.crservo3.setPower(-0.5);

            robot.crservo2.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.crservo4.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.crservo3.setDirection(DcMotorSimple.Direction.REVERSE);
            // spinSet4();

        }

        if (!B2isPressed && !A2isPressed) {

            robot.crservo2.setPower(0);
            robot.crservo3.setPower(0);

            robot.crservo4.setPower(0);
        }

        if (ServoBeaconUp) {

            position += INCREMENT;
            robot.servo1.setPosition(position);

        }
        // this does the same as servoUp goes to same spot - once button is relesed it's dones
        else if (ServoBeaconDown) {

            position -= INCREMENT;
            robot.servo1.setPosition(position);


        }


    }


    @Override
    public void stop() {


    }

}
