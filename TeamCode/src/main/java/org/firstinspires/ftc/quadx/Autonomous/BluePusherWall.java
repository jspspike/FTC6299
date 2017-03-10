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
        hold.setPosition(HOLD_HOLD);
        winch.setPower(0);

        delay(delay * 1000);

        flywheel.setPower(flyPow);
        manip.setPower(.3);
        moveTo(.35, 1560, .6, 1.5);
        delay(500);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurnPID(.3, 50, 4000);
        moveTo(.35, 5250, 6, 1.5);
        manip.setPower(0);
        arcTurnPID(-.37, -33, 2000);
        manip.setPower(.3);
        untilWhiteAlign(.3, .16, 1420, 5000);
        resetGyro();
        manip.setPower(0);
        if (!fail)
            moveTo(-.2, 170, .6, 1.5);
        pressBlue();
        untilWhiteAlign(-.3, -.16, 1950, 6150);
        if (!fail)
            moveTo(.2, 150, .6, 1.5);
        pressBlue();
        manip.setPower(.3);
        arcTurnPID(.3, -68, 3000);
        moveTo(.35, 2700, .6, 1.5);
        manip.setPower(0);
        arcTurnPID(.4, -60, 2000);


        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}