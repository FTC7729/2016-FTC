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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Autonomous POV", group="Pushbot")
//@Disabled - so we can see what we are doing
public class PushbotTeleopPOV_Linear_Auto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    double          clawOffset      = 0;                       // Servo mid position
    private double scoopUp;
    private double scoopDown;
    private double servroMe;
    public final static double SERVO_HOME = 0.2;
    public final static double SERVO_MIN_RANGE  = 0.20;
    public final static double SERVO_MAX_RANGE  = 0.90;
    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARDS_SPEED    = -0.6;



    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double max;
        double scoop;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver -< I'm in POV_Auto");    //
        telemetry.update();
        // for autonomous mode
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        //left = -gamepad1.left_stick_y + gamepad1.right_stick_x;
        //right = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        // we know the left_stick_x works
        //scoopUp = -gamepad1.left_stick_x;
        // we know the gamepad1.right_trigger - more detail control works
        //scoopUp = gamepad1.right_trigger;
        // we know the gamepad1.left_trigger - more detail control works
        //scoopUp = gamepad1.left_trigger;
        //scoopUp = -gamepad1.left_trigger;
        //scoopDown = gamepad1.left_stick_x;
       // servroMe = -gamepad1.right_stick_x;





        // Wait for the game to start (driver presses PLAY)


        waitForStart();

        DriveForwardTime(FORWARD_SPEED, 1000);


        DriveRightTime(FORWARD_SPEED, 1000);

        DriveLeftTime(FORWARD_SPEED, 1000);

        DriveLeftTime(FORWARD_SPEED, 1000);

        DriveBackTime(FORWARD_SPEED, 1000);

// setting values and motors

        //robot.leftMotor.setPower(1);
        //robot.rightMotor.setPower(1);
        //adding back motors
        //robot.leftMotorBack.setPower(1);
       // robot.rightMotorBack.setPower(1);


    // Where do we want to go in Auto mode?
        // Need to sleep in between? Thread.sleep()
        // Can we make variables here?

        // go forward?
       // DriveForward(FORWARD_SPEED);

        // go back?
       // DriveBack(BACKWARDS_SPEED);

        // go right?
       // TurnMeRight(FORWARD_SPEED);


        //go left?
        //TurnMeLeft(FORWARD_SPEED);

        // stopping
        DriveStop();
        
        // combine time and turn together
       // DriveForwardTime(1, 4000);



        // run until the end of the match (driver presses STOP)
       //We are commenting out the while loop to not make it complicated
/*
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = -gamepad1.left_stick_y + gamepad1.right_stick_x;
            right = -gamepad1.left_stick_y - gamepad1.right_stick_x;
            // we know the left_stick_x works
            //scoopUp = -gamepad1.left_stick_x;
            // we know the gamepad1.right_trigger - more detail control works
            //scoopUp = gamepad1.right_trigger;
            // we know the gamepad1.left_trigger - more detail control works
            //scoopUp = gamepad1.left_trigger;
            scoopUp = -gamepad1.left_trigger;
            //scoopDown = gamepad1.left_stick_x;
            servroMe = -gamepad1.right_stick_x;

=
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            //adding back motors
            robot.leftMotorBack.setPower(left);
            robot.rightMotorBack.setPower(right);

            robot.spinMotor.setPower(scoopUp);

        //working on Servo - still not working - hw problem?

            if (gamepad1.y) {
                robot.servo1.setPosition(SERVO_HOME);
            }

            if (gamepad1.a) {
                robot.servo1.setPosition(SERVO_MAX_RANGE);
            }

            // Use gamepad left & right Bumpers to open and close the claw


            // Move both servos to new position.  Assume servos are mirror image of each other.

            // Use gamepad buttons to move arm up (Y) and down (A)


            // Send telemetry message to signify robot running;
            telemetry.addData("claw", "Offset = %.2f", clawOffset);
            telemetry.addData("left", "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("scoopUp", "%.2f", scoopUp);
            telemetry.addData("servo1", "%.2f", servroMe);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
*/
    }

    private void DriveForwardTime(double power, long time) throws InterruptedException{

        DriveForward(power);
        Thread.sleep(time);
        // the thread.sleep could throw an exception -> for later times
    }

    private void DriveBackTime(double power, long time) throws InterruptedException{

        DriveBack(power);
        Thread.sleep(time);
        // the thread.sleep could throw an exception -> for later times
    }

    private void DriveRightTime(double power, long time) throws InterruptedException{

        TurnMeRight(power);
        Thread.sleep(time);
        // the thread.sleep could throw an exception -> for later times
    }

    private void DriveLeftTime(double power, long time) throws InterruptedException{

        TurnMeLeft(power);
        Thread.sleep(time);
        // the thread.sleep could throw an exception -> for later times
    }

    private void TurnMeRight(double power) {
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(-power);
        //adding back motors
        robot.leftMotorBack.setPower(power);
        robot.rightMotorBack.setPower(-power);
    }

    private void TurnMeLeft(double power) {
        robot.leftMotor.setPower(-power);
        robot.rightMotor.setPower(power);
        //adding back motors
        robot.leftMotorBack.setPower(-power);
        robot.rightMotorBack.setPower(power);

    }

    private void DriveStop() {
        // if driveForward is 0, this means the bot has stopped
        DriveForward(0);
    }

    private void DriveBack(double power) {
        robot.leftMotor.setPower(-power);
        robot.rightMotor.setPower(-power);
        //adding back motors
        robot.leftMotorBack.setPower(-power);
        robot.rightMotorBack.setPower(-power);
    }

    private void DriveForward(double power) {
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);
        //adding back motors
        robot.leftMotorBack.setPower(power);
        robot.rightMotorBack.setPower(power);

    }
}