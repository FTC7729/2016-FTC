package chawks.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.lasarobotics.vision.ftc.resq.Beacon;

import chawks.hardware.Dutchess;

import static org.lasarobotics.vision.opmode.VisionOpMode.beacon;

/**
 * Created by joseph on 1/21/17.
 */

@Autonomous(name = "move to beacon", group = "Pushbot")
public class AbstractShootAndMoveAuto extends AbstractAutonomous {

    private final Dutchess robot = new Dutchess();

    @Override
    public void runMovement() {
        //encoderDrive(double speed, double feetDistance, double timeoutS)
        encoderDrive(.6, 2.0, 1.0);

        //encoderDriveDirect(double speed, double leftFeet, double rightFeet, double leftBackFeet, double rightBackFeet, double timeoutS)
        encoderDriveDirect(.6, (3.4 * Math.PI / 12), -(3.4 * Math.PI / 12), (3.4 * Math.PI / 12), -(3.5 * Math.PI / 12), .95);

        //encoderDrive(double speed, double feetDistance, double timeoutS)
        encoderDrive(.6, 1.0, 1.3); // switched to 1 ft so robot can get a good look at the beacon


    }






}
