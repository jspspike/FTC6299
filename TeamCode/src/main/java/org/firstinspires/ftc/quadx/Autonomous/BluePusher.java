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

        resetGyro();

        while (!gamepad1.a && !opModeIsActive()) {
            telemetry.addData("gyro", getGyroYaw());
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro", "Completed");
        telemetry.update();

        double flyPow = flyPow();
        int moveVal = encoderPow();

        waitForStart();


        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);

        setManip(-.3);

        moveTo(.25, moveVal, .6, 1.5);
        arcTurnCorr(.5, -44.3);
        untilWhite(.15, .15, 100, 2100);
        if (!fail)
            moveTo(.2, -140, .6, 1.5);
        pressBlue();

        untilWhiteRange(.35, .15, 15, 1450, 5930);
        if (!fail)
            moveTo(.2, -200, .6, 1.5);
        setManip(0);
        pressBlue();

        moveTo(.2, -600);
        arcTurn(.45, -70, false);
        arcTurn(.5, -61 );
        setFly(flyPow);
        moveTo(.2, 2520, 6);
        delay(4000);
        door.setPosition(DOOR_OPEN);
        delay(1100);
        moveTo(.2, 1600, 6);
    }
}
