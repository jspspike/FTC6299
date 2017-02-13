package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Blue Pusher Wall", group="Blue")
public class BluePusherWall extends MyOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        initServos();
        initSensors();

        resetGyro();

        int delay = 0;
        double flyPow = .66;

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

        delay(delay);

        flywheel.setPower(flyPow);
        moveTo(.35, 1500, .6, 1.5);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurn(.45, 25);
        moveAlign(.4, .2, 8000, 12000);
        untilWhiteAlign(-.3, -.15, 0, 3000, .5, false, true);
        if (!fail)
            moveTo(.2, 170, .6, 1.5);
        untilWhiteAlign(-.3, -.15, 2000, 5000, .5, false, false);
        if (!fail)
            moveTo(.2, 170, .6, 1.5);
    }
}