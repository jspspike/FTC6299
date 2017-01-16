package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 1/15/17.
 */

@Autonomous(name="Blue PUsher Long", group="Blue")
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

        double flyPow = flyPow();
        int moveVal = encoderPow() + 2000;
        double turnVal;

        waitForStart();

        turnVal = getGyroYaw();


        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);

        setManip(-.3);

        moveTo(.25, moveVal, .6, 1.5);
        arcTurnCorr(.5, turnVal * -1 + .7);
        untilWhite(.15, .15, 100, 2500);
        if (!fail)
            moveTo(.2, -140, .6, 1.5);
        pressBlue();

        untilWhiteRange(-.35, -.15, -15, -1600, -6430);
        if (!fail)
            moveTo(.2, 200, .6, 1.5);
        setManip(0);
        pressBlue();

        arcTurn(.45, -85, false);
        setFly(flyPow);
        moveTo(.2, 2520, 6);
        delay(1000);
        door.setPosition(DOOR_OPEN);
        delay(1100);
        moveTo(.2, 1600, 6);
    }
}
