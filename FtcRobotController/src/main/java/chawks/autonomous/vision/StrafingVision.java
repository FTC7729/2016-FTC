package chawks.autonomous.vision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import chawks.hardware.Dutchess;

@Autonomous(name = "Color (speed=0.5)", group = "Pushbot")
public class StrafingVision extends AbstractVisionOpMode {
    /**
     * Joe: Did some testing, and seems to require a value of at least 0.4f
     */
    public static final float STAFE_SPEED = 0.5f;

    public StrafingVision(Dutchess robot) {
        super(robot, STAFE_SPEED);
    }
}

