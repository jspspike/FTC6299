package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Blue Pusher", group="Blue")
public class BluePusher extends MyOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();
        waitForStart();
        floorL.enableLed(true);
        floorR.enableLed(true);


        arcTurnCorr(.2, 45);
        moveTo(.2, 4800);
        arcTurnCorr(.2, -46);
        untilWhite(.12);
        moveTo(.2, -100);
        pressBlue();
        untilWhite(.12);
        moveTo(.2, -100);
        pressBlue();
    }
}
