package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Red Pusher Wall", group="Red")
public class RedPusherWall extends MyOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        initServos();
        initSensors();

        resetGyro();

        int delay = 0;
        double flyPow = .627;

        while (!opModeIsActive()) {

            if (gamepad1.dpad_left) {
                delay -= 1;
                delay(250);
            }
            else if (gamepad1.dpad_right) {
                delay += 1;
                delay(250);
            }

            telemetry.addData("Delay", delay);
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.update();
            idle();
        }

        telemetry.addData("Gyro", "Completed");
        telemetry.update();

        waitForStart();


        winch.setPower(-1);
        delay(1000);
        winch.setPower(0);
        hold.setPosition(HOLD_HOLD);

        delay(delay * 1000);

        flywheel.setPower(flyPow);
        moveTo(.35, 1520, .6, 1.5);
        delay(500);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurnPID(-.3, 65);
        arcTurnPID(-.3, 60, 4000);
        moveTo(-.35, 4900, 6, 1.5);
        arcTurnPID(.38, 30);
        untilWhiteAlign(-.3, -.15, 1800, 5700);
        if (!fail)
            moveTo(.2, 140, .6, 1.5);
        pressRed();
        untilWhiteAlign(.3, .15, 2300, 6100);
        if (!fail)
            moveTo(-.2, 140, .6, 1.5);
        pressRed();
        moveTo(.4, -750);
        arcTurnPID(.4, -50);
        moveTo(.35, 2500, .6, 1.5);
        arcTurnPID(.4, 55);

        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}