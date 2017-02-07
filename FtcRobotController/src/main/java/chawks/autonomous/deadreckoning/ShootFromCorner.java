package chawks.autonomous.deadreckoning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot from corner", group = "Pushbot")
public class ShootFromCorner extends AbstractDeadReckoningOpMode {
    //encoderDrive(double speed, double feetDistance, double timeoutS)
    //encoderDriveDirect(double speed, double leftFeet, double rightFeet, double leftBackFeet, double rightBackFeet, double timeoutS)

    @Override
    public void initMovement() {
        //drive into shooting range
        encoderDrive(.6, 1.0, .5);

        // end method : "return"
        return;
    }

    public void runMovement() {
        //drive forward into cap ball
        encoderDrive(.6, 1.0, .5);

        // end method : "return"
        return;
    }

}
