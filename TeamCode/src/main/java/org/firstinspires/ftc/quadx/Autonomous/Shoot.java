package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Shoot", group="Blue")
public class Shoot extends MyOpMode {

    long delay = 8;
    long ballDelay = 5;
    int cap = 0;
    String capString = "";


    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();

        double flyPow = .65;

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
            if (gamepad1.x){
                cap = 2;
            }

            if (cap == 0){
                capString = "cap";
            } else if(cap == 1){
                capString = "block";
            } else if(cap == 2){
                capString = "ramp";
            }


            telemetry.addData("Delay", delay);
            telemetry.addData("Ball Delay", ballDelay);
            telemetry.addData("Hit ball", cap);
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.addData("Cap", capString);
            telemetry.update();
            idle();
        }

        waitForStart();

        delay(delay * 1000);

        arcTurn(.55, 40);
        flywheel.setPower(flyPow);
        moveTo(.2, 2700, 8);
        delay(2000);
        door.setPosition(DOOR_OPEN);
        delay(4000);
        door.setPosition(DOOR_CLOSED);
        flywheel.setPower(0);
        if (cap == 0) {
            delay(ballDelay * 1000);
            moveTo(.4, 3600);
        } else if (cap == 1) {

        } else if (cap == 2) {

        }
    }
}
