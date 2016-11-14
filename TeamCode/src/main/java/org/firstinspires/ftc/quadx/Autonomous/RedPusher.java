package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Red Pusher", group="Red")
public class RedPusher extends MyOpMode {

    double startingVoltage;
    double flyPow;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();
<<<<<<< 226031f6d581d9f0f53ff65c02fa47f678272c98
        int moveVal = -4670;

        resetGyro();


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

        while (!gamepad1.a && opModeIsActive()) {
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.update();
            idle();
        }


        telemetry.addData("Gyro", "Finished");
        telemetry.update();

        waitForStart();

        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.8) {
            moveVal = -4780;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.6) {
            moveVal = -4775;
        }
        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5){
            moveVal = -4690;
        }
=======
        int moveVal = -4650;

        waitForStart();


        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5) {
            moveVal = -4720;
        }

>>>>>>> Finalized autonomous
        floorL.enableLed(true);
        floorR.enableLed(true);

        arcTurnCorr(-.2, -45);
        moveTo(.2, moveVal);
<<<<<<< 226031f6d581d9f0f53ff65c02fa47f678272c98
        arcTurnCorr(-.2, 44.3);
        untilWhiteRange(-.2, 13, -150);
        moveTo(.2, 150);
        pressRed();
        untilWhiteRange(-.2, 13, -1000);
        moveTo(.2, 230);
//        flywheel.setPower(flyPow);
=======
        arcTurnCorr(-.2, 45);
        untilWhite(-.12);
        pressRed();
        arcTurn(-.2, .7);
        untilWhite(-.12);
        moveTo(.2, 50);
>>>>>>> Finalized autonomous
        pressRed();
//        arcTurn(-.2, -45);
//        moveTo(.2, 2500);
//        door.setPosition(DOOR_OPEN);
//        delay(500);
//        door.setPosition(DOOR_CLOSED);
//        delay(1300);
//        door.setPosition(DOOR_OPEN);
//        delay(500);
//        door.setPosition(DOOR_CLOSED);
//        delay(1000);
//        flywheel.setPower(0);
    }
}
