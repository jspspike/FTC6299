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
        double flyPow = .625;

        double encoderDis = 5100;

        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.7) {
            encoderDis = 5150;
            flyPow = .625;
        }

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


        winch.setPower(1);
        delay(1000);
        winch.setPower(0);
        hold.setPosition(HOLD_HOLD);

        delay(delay * 1000);

        flywheel.setPower(flyPow);
        moveTo(.35, 1500, .6, 1.5);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurn(-.55, 65);
        arcTurn(-.55, 60);
        moveTo(-.35, 5150, .6, 1.5);
        arcTurn(-.55, 45);
        untilWhiteAlign(-.3, -.15, 2000, 5500);
        if (!fail)
            moveTo(.2, 170, .6, 1.5);
        pressRed();
        untilWhiteAlign(.3, .15, 3000, 6000);
        if (!fail)
            moveTo(-.2, 170, .6, 1.5);
        pressRed();
        arcTurn(.4, -90);
        moveTo(.35, 2500, .6, 1.5);
        arcTurn(.4, 80);

        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}