package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Shoot Red", group="Red")
public class ShootRed extends MyOpMode {

    long delay = 8;
    long ballDelay = 5;
    int cap = 0;
    String capString = "";

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();

        while (!opModeIsActive()) {

            if (gamepad1.dpad_left) {
                delay -= 1;
                delay(250);
            }
            else if (gamepad1.dpad_right) {
                delay += 1;
                delay(250);
            }

            if (gamepad1.left_bumper) {
                ballDelay -= 1;
                delay(250);
            }
            else if (gamepad1.right_bumper) {
                ballDelay += 1;
                delay(250);
            }

            if (gamepad1.a){
                cap = 0;
            }
            if (gamepad1.b){
                cap = 1;
            }
            if(gamepad1.x){
                cap = 2;
            }

            if (cap == 0) {
                capString = "center";
            } else if (cap == 1){
                capString = "block";
            } else if (cap == 2){
                capString = "ramp";
            }
            telemetry.addData("Delay", delay);
            telemetry.addData("Ball Delay", ballDelay);
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.addData("Cap", capString);
            telemetry.update();
            idle();
        }

        double flyPow = .64;
        waitForStart();

        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();


        telemetry.addData("Volatage", startingVoltage);


        delay(delay * 1000);
        arcTurnPID(.4, -36, 1800);
        flywheel.setPower(flyPow);
        moveTo(.2, 3100, 8);
        delay(2000);
        door.setPosition(DOOR_OPEN);
        delay(4000);
        door.setPosition(DOOR_CLOSED);
        flywheel.setPower(0);

        if (cap == 1) {
            gyroError = 0;
            arcTurnPID(-.35, 80, 1700);
            moveTo(.3, 5500, .6, 1.5);
            arcTurnPID(.35, -80, 1700);
            moveTo(.3, 3800, .6, 1.5);
            arcTurnPID(.35, -45, 1700);
            moveTo(.3, 2700, .6, 1.5);
        } else if (cap == 2){
            gyroError = 0;
            arcTurnPID(-.35, -63, 1700);
            moveToSlow(.5, 8050, .6, 1.5, 5000, true);
        } else {
            delay(ballDelay * 1000);
            moveTo(.4, 3600);
        }
    }
}
