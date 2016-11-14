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
        int moveVal = -4650;

        waitForStart();


        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5) {
            moveVal = -4720;
        }

        floorL.enableLed(true);
        floorR.enableLed(true);


        arcTurnCorr(-.2, -45);
        moveTo(.2, moveVal);
        arcTurnCorr(-.2, 45);
        untilWhite(-.12);
        pressRed();
        arcTurn(-.2, .7);
        untilWhite(-.12);
        moveTo(.2, 50);
        pressRed();
    }
}
