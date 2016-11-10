package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/3/16.
 */


@Autonomous(name="Blue Pusher", group="Blue")
public class BluePusher extends MyOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();
        
        int moveVal = 4640;

        resetGyro();

        while (!gamepad1.a && opModeIsActive()) {
            telemetry.addData("gyro", getGyroYaw());
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro", "Completed");
        telemetry.update();

        waitForStart();


        if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.7) {
            moveVal = 4761;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.65) {
            moveVal = 4745;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.6) {
            moveVal = 4720;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.55) {
            moveVal = 4680;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.5) {
            moveVal = 4660;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.4) {
            moveVal = 4635;
        }

        else if (hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage() > 13.2) {
            moveVal = 4640;
        }

        telemetry.addData("MoveVal", moveVal);
        telemetry.update();

        floorL.enableLed(true);
        floorR.enableLed(true);

        manip.setPower(-.3);
        arcTurnCorr(.2, 45);
        moveTo(.2, moveVal, .6, 1.5);
        arcTurnCorr(.2, -44.3);
        untilWhiteRange(.2, 13, 100);
        moveTo(.2, -135, .6, 1.5);
        pressBlue();
//        arcTurn(-.2, .7);
        untilWhiteRange(.2, 13, 1000);
        moveTo(.2, -200, .6, 1.5);
        pressBlue();
    }
}
