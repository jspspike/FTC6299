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
        int moveVal = 4680;

        waitForStart();


        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.7) {
            moveVal = 4750;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.6) {
            moveVal = 4730;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5) {
            moveVal = 4660;
        }

        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);


        arcTurnCorr(.2, 45);
        moveTo(.2, moveVal, .6, 1.5);
        arcTurnCorr(.2, -45);
        untilWhite(.2);
        moveTo(.2, -285, .6, 1.5);
        pressBlue();
        moveTo(.15, 400, .6, 1.5);
//        if (moveVal == 4825)
//            arcTurn(-.2, -.7);
//        else
//            arcTurn(-.2, -.7);
        untilWhite(.2);
        moveTo(.2, -285, .6, 1.5);
        pressBlue();
    }
}
