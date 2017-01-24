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

        resetGyro();

        while (!gamepad1.a && !opModeIsActive()) {
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.update();
            idle();
        }

        telemetry.addData("Gyro", "Finished");
        telemetry.update();

        double flyPow = .66;
        int moveVal = -(encoderPow() + 70);

        waitForStart();

        floorL.enableLed(true);
        floorR.enableLed(true);


        moveTo(.25, moveVal);
        arcTurnCorr(-.5, 44.3);
        untilWhite(-.15, -.15, -100, -1900);
        if (!fail)
            moveTo(.2, 100);
        pressRed();

        untilWhiteRange(-.35, -.15, 15, -1000, -5830);
        if (!fail)
            moveTo(.2, 230);
        pressRed();

        moveTo(.3, 325);
        arcTurn(.37, -41.5);
        setFly(flyPow);
        moveTo(.25, 3150, 6);
        delay(1000);
        door.setPosition(DOOR_OPEN);
        delay(1100);
        moveTo(.2, 1900, 6);
    }
}
