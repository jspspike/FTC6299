package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Shoot", group="Blue")
public class Shoot extends MyOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();

        double flyPow = flyPow();

        waitForStart();

        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();


        telemetry.addData("Volatage", startingVoltage);

        flywheel.setPower(flyPow);

        arcTurn(.2, 40);
        moveTo(.2, 2800, 8);
        delay(7000);
        door.setPosition(DOOR_OPEN);
        delay(500);
        door.setPosition(DOOR_CLOSED);
        delay(1500);
        door.setPosition(DOOR_OPEN);
        delay(900);
        door.setPosition(DOOR_CLOSED);
        delay(1000);
        flywheel.setPower(0);

        moveTo(.4, 3800);
    }
}
