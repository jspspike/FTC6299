package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Shoot", group="Blue")
public class Shoot extends MyOpMode {


    public static final int POLL_RATE = 40;


    double flyPow = 0.0;
    double oldFly = 0.0;
    double flyRPM = 0;
    int rpmValCount = 0;
    double[] rpmVals = new double[POLL_RATE];
    double rpmAvg;
    double desiredRPM = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();

        waitForStart();

        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();


        telemetry.addData("Volatage", startingVoltage);

        if (startingVoltage >= 13.8) {
            flyPow = -.3;
        }

        else if (startingVoltage >= 13.5) {
            flyPow = -.31;
        }

        else if (startingVoltage >= 13.3) {
            flyPow = -.33;
        }

        else if (startingVoltage >= 13) {
            flyPow = -.34;
        }

        else if (startingVoltage >= 12.7) {
            flyPow = -.35;
        }

        else if (startingVoltage >= 12.5) {
            flyPow = -.37;
        }

        else if (startingVoltage >= 12.3) {
            flyPow = -.39;
        }

        else {
            flyPow = -.4;
        }

//        flyRPM = (Math.abs(flywheel.getCurrentPosition()) - oldFly) / getRuntime();
//
//        oldFly = Math.abs(flywheel.getCurrentPosition());
//
//        if (rpmValCount > POLL_RATE - 1) {
//            rpmAvg = 0;
//            for (int i = 0; i < rpmVals.length; i++) {
//                rpmAvg += rpmVals[i];
//            }
//
//            rpmAvg /= POLL_RATE;
//            rpmValCount = 0;
//        }


        flywheel.setPower(flyPow);
        moveTo(.2, 1400);
        delay(10000);
        door.setPosition(DDOR_OPEN);
        delay(500);
        door.setPosition(DOOR_CLOSED);
        delay(1300);
        door.setPosition(DDOR_OPEN);
        delay(500);
        door.setPosition(DOOR_CLOSED);
        delay(1000);
        flywheel.setPower(0);

        moveTo(.4, 5000);
    }
}
