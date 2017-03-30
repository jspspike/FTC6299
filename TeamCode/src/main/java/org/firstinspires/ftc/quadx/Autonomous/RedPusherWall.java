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
        int block = 0;
        String blockStat = "";

        while (!opModeIsActive()) {

            if (gamepad1.dpad_left) {
                delay -= 1;
                delay(250);
            }
            else if (gamepad1.dpad_right) {
                delay += 1;
                delay(250);
            }

            if(gamepad1.a){
                block = 0;
            }
            if(gamepad1.b){
                block = 1;
            }
            if(gamepad1.x){
                block = 2;
            }

            if(block == 0){
                blockStat = "center";
            } else if (block == 1) {
                blockStat = "block";
            } else if (block == 2) {
                blockStat = "ramp";
            }



            telemetry.addData("Delay", delay);
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.addData("Block", blockStat);
            telemetry.update();
            idle();
        }

        telemetry.addData("Gyro", "Completed");
        telemetry.update();

        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.85) {
            flyPow = .639;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.6) {
            flyPow = .643;
        }

        else {
            flyPow = .647;
        }

        waitForStart();

        winch.setPower(-1);
        delay(1000);
        winch.setPower(0);
        hold.setPosition(HOLD_HOLD);

        delay(delay * 1000);

        flywheel.setPower(flyPow);
        manip.setPower(.3);
        moveTo(.35, 1900, .6, 1.5);
        manip.setPower(0);
        delay(500);
        door.setPosition(DOOR_OPEN);
        delay(2000);
        flywheel.setPower(0);
        arcTurnPID(-.35, 70, 1500);
        arcTurnPID(-.35, 68, 1500);
        moveToSlow(-.35, 5930, 6, 1.5, 6000, true);
        arcTurnPID(.38, 36, 2000);
        untilWhiteAlign(-.35, -.15, 1800, 6000);
        resetGyro();
        if (!fail)
            moveTo(.2, 140, .6, 1.5);
        pressRed();
        manip.setPower(.3);
        untilWhiteAlign(.3, .15, 2300, 6100);
        manip.setPower(0);
        if (!fail)
            moveTo(-.2, 170, .6, 1.5);
        pressRed();
        if(block == 1){
            manip.setPower(.3);
            arcTurnPID(-.4, 60, 2000);
            moveTo(.3, -5000, .6, 1.5);
            manip.setPower(0);
        } else if(block == 2){
            arcTurnPID(.45, -30, 2000);
            moveTo(.3, 2600, .6, 1.5, 4000, true);
            manip.setPower(0);
        }  else {
            moveTo(.4, -790);
            manip.setPower(.3);
            arcTurnPID(.4, -80, 3000);
            moveTo(.35, 2500, .6, 1.5);
            manip.setPower(0);
            arcTurnPID(.4, 55, 2000);
        }

        winch.setPower(1);
        hold.setPosition(HOLD_DISABLED);
        delay(1000);
        winch.setPower(0);
    }
}