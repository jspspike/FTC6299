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
        int moveVal = 4650;

        waitForStart();


        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.7) {
            moveVal = 4750;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.6) {
            moveVal = 4700;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5) {
            moveVal = 4660;
        }

        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);


        arcTurnCorr(.2, 45);
        moveTo(.2, moveVal);
        arcTurnCorr(.2, -45);
        untilWhite(.12);
        moveTo(.2, -140);
        pressBlue();
        moveTo(.15, 400);
        if (moveVal == 4825)
            arcTurn(-.2, -.7);
        else
            arcTurn(-.2, -.7);
        untilWhite(.12);
        moveTo(.2, -145);
        pressBlue();
    }
}
