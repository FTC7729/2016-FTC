package chawks.autonomous.deadreckoning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by karahandy on 2/17/17.
 */
@Autonomous (name = "Shoot From Wall", group = "Pushbot")
public class ShootFromWall extends AbstractDeadReckoningOpMode {

    @Override
    public void runMovement() {

        shoot();

    }



}
