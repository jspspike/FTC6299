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
        double flyPow = .633;

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

        double encoderDis = 4970;

        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.7) {
            encoderDis = 4950;
            flyPow = .63;
        }

        waitForStart();

        winch.setPower(-1);
        delay(1000);
        hold.setPosition(HOLD_HOLD);
        winch.setPower(0);

        delay(delay * 1000);

        flywheel.setPower(flyPow);
        moveTo(.35, 1520, .6, 1.5);
        delay(500);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurn(.55, 47);
        moveTo(.35, encoderDis, 6, 1.5);
        arcTurn(.53, -40);
        untilWhiteAlign(.3, .17, 1850, 4500);
        if (!fail)
            moveTo(-.2, 210, .6, 1.5);
        pressBlue();
        untilWhiteAlign(-.3, -.15, 1950, 6150);
        if (!fail)
            moveTo(.2, 190, .6, 1.5);
        pressBlue();
        arcTurn(.4, -90);
        moveTo(.35, 2700, .6, 1.5);
        arcTurn(.4, -80);

        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}