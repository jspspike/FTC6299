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

        double flyPow = flyPow();
        int moveVal = -(encoderPow() + 40);

        waitForStart();

        floorL.enableLed(true);
        floorR.enableLed(true);


        moveTo(.2, moveVal);
        arcTurnCorr(-.5, 44.3);
        untilWhite(-.15, -.15);
        moveTo(.2, 100);
        pressRed();

        untilWhiteRange(-.35, -.15, 14, -1000);
        moveTo(.2, 230);
        pressRed();

        arcTurn(.35, -46);
        flywheel.setPower(flyPow);
        moveTo(.25, 2800, 6);
        delay(3500);
        door.setPosition(DOOR_OPEN);
        delay(1000);
        flywheel.setPower(0);

        moveTo(.2, 1800, 6);
    }
}
