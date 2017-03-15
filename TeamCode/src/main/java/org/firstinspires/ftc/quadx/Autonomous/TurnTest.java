package org.firstinspires.ftc.quadx.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

/**
 * Created by jspspike on 3/1/17.
 */
@Autonomous(name = "Turn Test", group = "Testing")

public class TurnTest extends MyOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        initServos();
        initSensors();

        waitForStart();

        arcTurnPID(-.3, -45);

        while(opModeIsActive()) {
            telemetry.addData("Gryo", getGyroYaw());
            telemetry.update();
        }
    }
}
