package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 11/1/16.
 */

@Autonomous(name="Sensor Test", group="Testing")
public class SensorTest extends MyOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initSensors();
        resetGyro();
        waitForStart();
        floorL.enableLed(true);
        floorR.enableLed(true);


        while(opModeIsActive()) {
            telemetry.addData("FloorL", floorL.alpha());
            telemetry.addData("FloorR", floorR.alpha());
            telemetry.addData("BeaconL Red", beaconL.red());
            telemetry.addData("BeaconL Blue", beaconL.blue());
            telemetry.addData("BeaconR Red", beaconR.red());
            telemetry.addData("BeaconR Blue", beaconR.blue());
            telemetry.addData("Gryo Raw", gyro.getAngularOrientation().firstAngle);
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.update();
        }
    }
}
