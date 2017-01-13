package org.firstinspires.ftc.robotcontroller.internal;

import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

/**
 * {@link FtcOpModeRegister} is responsible for registering opmodes for use in an FTC game.
 *
 * @see #register(OpModeManager)
 */
public class FtcOpModeRegister implements OpModeRegister {
    public void register(OpModeManager manager) {
        BlocksOpMode.registerAll(manager);
        AnnotatedOpModeRegistrar.register(manager);
    }
}
