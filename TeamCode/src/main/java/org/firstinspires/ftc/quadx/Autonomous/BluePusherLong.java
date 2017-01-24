package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Blue Pusher Long", group="Blue")
public class BluePusherLong extends MyOpMode {
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

        double flyPow = .66;
        int moveVal = encoderPow();

        waitForStart();


        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);

        setManip(-.3);

        moveTo(.25, moveVal, .6, 1.5);
        arcTurnCorr(.5, -44.3);
        untilWhiteRange(.35, .15, 15, 1800, 7000);
        if (!fail)
            moveTo(.2, -200, .6, 1.5);
        pressBlue();

        untilWhiteRange(-.35, -.15, 15, 1500, 7000);
        if (!fail)
            moveTo(.2, 200, .6, 1.5);
        setManip(0);
        pressBlue();

        arcTurn(.45, -85);
        flywheel.setPower(flyPow);
        moveTo(.2, 2050, 6);
        delay(1000);
        door.setPosition(DOOR_OPEN);
        delay(1100);
        moveTo(.2, 1600, 6);
    }
}