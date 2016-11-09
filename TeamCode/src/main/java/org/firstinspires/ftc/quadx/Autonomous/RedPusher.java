package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Red Pusher", group="Red")
public class RedPusher extends MyOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();
        waitForStart();
        floorL.enableLed(true);
        floorR.enableLed(true);


        arcTurnCorr(-.3, -45);
        moveTo(.3, -4700);
        arcTurnCorr(-.3, 45);
        untilWhite(-.15);
//        moveTo(.15, 200);
//        untilWhite(.15);
        pressRed();
    }
}
