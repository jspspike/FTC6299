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
        int moveVal = 4720;

        resetGyro();

        while (!gamepad1.a && !opModeIsActive()) {
            telemetry.addData("gyro", getGyroYaw());
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro", "Completed");
        telemetry.update();

        double flyPow = flyPow();

        waitForStart();


        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);

        manip.setPower(-.3);
        moveTo(.25, moveVal, .6, 1.5);
        arcTurnCorr(.4, -44.3);
        untilWhite(.3, .15, 200);
        moveTo(.2, -100, .6, 1.5);
        pressBlue();
//        arcTurn(-.2, .7);
        untilWhiteRange(.35, .15, 14, 1450);
        moveTo(.2, -200, .6, 1.5);
        manip.setPower(0);
        pressBlue();
        moveTo(.2, -600);
        arcTurn(.35, -132);
        flywheel.setPower(flyPow);
        moveTo(.2, 2300, 6);
        delay(3500);
        door.setPosition(DOOR_OPEN);
        delay(1000);
        flywheel.setPower(0);
        moveTo(.2, 1600, 6);
    }
}
