package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Shoot Red", group="Red")
public class ShootRed extends MyOpMode {


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
            flyPow = -.44;
        }

        else if (startingVoltage >= 13.5) {
            flyPow = -.45;
        }

        else if (startingVoltage >= 13.3) {
            flyPow = -.46;
        }

        else if (startingVoltage >= 13) {
            flyPow = -.47;
        }

        else if (startingVoltage >= 12.7) {
            flyPow = -.48;
        }

        else if (startingVoltage >= 12.5) {
            flyPow = -.49;
        }

        else if (startingVoltage >= 12.3) {
            flyPow = -.51;
        }

        else {
            flyPow = -.52;
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

        arcTurn(.2, -30);
        moveTo(.2, 3200, 8);
        delay(7000);
        door.setPosition(DOOR_OPEN);
        delay(500);
        door.setPosition(DOOR_CLOSED);
        delay(1500);
        door.setPosition(DOOR_OPEN);
        delay(900);
        door.setPosition(DOOR_CLOSED);
        delay(1000);
        flywheel.setPower(0);

        moveTo(.4, 3800);
    }
}
