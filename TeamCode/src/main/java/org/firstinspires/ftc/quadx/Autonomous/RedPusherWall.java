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

        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.85) {
            flyPow = .633;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.6) {
            flyPow = .638;
        }

        else {
            flyPow = .645;
        }

        waitForStart();

        winch.setPower(-1);
        delay(1000);
        winch.setPower(0);
        hold.setPosition(HOLD_HOLD);

        delay(delay * 1000);

        flywheel.setPower(flyPow);
        manip.setPower(.3);
        moveTo(.35, 1560, .6, 1.5);
        manip.setPower(0);
        delay(500);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurnPID(-.3, 60, 2600);
        arcTurnPID(-.35, 62, 3000);
        moveToSlow(-.35, 6270, 6, 1.5);
        gyroError = 0;
        arcTurnPID(.38, 28, 2500);
        untilWhiteAlign(-.3, -.15, 1800, 6100);
        resetGyro();
        if (!fail)
            moveTo(.2, 140, .6, 1.5);
        pressRed();
        manip.setPower(.3);
        untilWhiteAlign(.3, .15, 2300, 6100);
        manip.setPower(0);
        if (!fail)
            moveTo(-.2, 150, .6, 1.5);
        pressRed();
        moveTo(.4, -750);
        manip.setPower(.3);
        arcTurnPID(.4, -80, 3000);
        moveTo(.35, 2500, .6, 1.5);
        manip.setPower(0);
        arcTurnPID(.4, 55, 2000);

        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}