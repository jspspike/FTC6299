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
        double turnDeg;

        double encoderDis = 4950;

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

        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.7) {
            encoderDis = 4950;
            flyPow = .625;
            turnDeg = 64;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5) {
            turnDeg = 64.5;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.3) {
            turnDeg = 64;
            flyPow = .627;
        }

        else  {
            turnDeg = 65;
            flyPow = .63;
        }

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
        arcTurn(-.55, 65);
        arcTurn(-.55, turnDeg);
        moveTo(-.35, encoderDis, 6, 1.5);
        arcTurn(-.55, 45);
        untilWhiteAlign(-.3, -.15, 1800, 5700);
        if (!fail)
            moveTo(.2, 140, .6, 1.5);
        pressRed();
        untilWhiteAlign(.3, .15, 2500, 6100);
        if (!fail)
            moveTo(-.2, 170, .6, 1.5);
        pressRed();
        moveTo(.4, -750);
        arcTurn(.4, -90);
        moveTo(.35, 2500, .6, 1.5);
        arcTurn(.4, 80);

        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}