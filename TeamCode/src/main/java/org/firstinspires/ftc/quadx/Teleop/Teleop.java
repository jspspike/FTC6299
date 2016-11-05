
package org.firstinspires.ftc.quadx.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="TeleOp", group="Teleop")
public class Teleop extends LinearOpMode {

    DcMotor fly;
    DcMotor manip;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor manipTop;

    Servo door;
    Servo buttonP;
    Servo manipPull;

    double flyPow = 0.0;
    double oldFly = 0.0;
    double flyRPM = 0;
    int rpmValCount = 0;
    double[] rpmVals = new double[POLL_RATE];
    double rpmAvg;
    double desiredRPM = 0;

    private ElapsedTime runtime = new ElapsedTime();

    public static final int POLL_RATE = 40;

    @Override
    public void runOpMode() throws InterruptedException {

        fly = hardwareMap.dcMotor.get("fly");
        manip = hardwareMap.dcMotor.get("manip");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        manipTop = hardwareMap.dcMotor.get("manipTop");

        door = hardwareMap.servo.get("door");
        buttonP = hardwareMap.servo.get("buttonP");
        manipPull = hardwareMap.servo.get("manipPull");

        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manipTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        door.setPosition(.2);
        buttonP.setPosition(.5);

        waitForStart();
        runtime.reset();


        /*Runnable flyWheel = new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad1.a){
                        fly.setPower(1);
                    }
                    else if (gamepad1.b) {
                        fly.setPower(-1);
                    }
                    else {
                        fly.setPower(0);
                    }
                    try {
                        idle();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        };

        Runnable manipulator = new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (gamepad2.a) {
                        manip.setPower(1);
                    }
                    else if (gamepad2.b) {
                        manip.setPower(-1);
                    }
                    else{
                        manip.setPower(0);
                    }

                    try {
                        idle();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                }
            }
        };


        Thread manipThread = new Thread(manipulator);
        manipThread.start();


        Thread flyWheelThread = new Thread(flyWheel);
        flyWheelThread.start();

        */

        resetStartTime();

        while (opModeIsActive()) {

            if (Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) {
                motorBL.setPower(gamepad1.left_stick_y);
                motorBR.setPower(-gamepad1.right_stick_y);
                motorFL.setPower(gamepad1.left_stick_y);
                motorFR.setPower(-gamepad1.right_stick_y);
            } else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            if (gamepad2.left_bumper)
                door.setPosition(.6);
            else if (gamepad2.right_bumper)
                door.setPosition(.2);

            if (gamepad2.a) {
                desiredRPM = 1600;
            }

            else if (gamepad2.b) {
                desiredRPM = 1315;
            }

            else if (gamepad2.x) {
                desiredRPM = 1450;
            }

            else if (gamepad2.y) {
                desiredRPM = 0;
            }

            if (gamepad2.dpad_left) {
                desiredRPM -= 10;
                Thread.sleep(200);
            }

            else if (gamepad2.dpad_right) {
                desiredRPM += 10;
                Thread.sleep(200);
            }

            if (gamepad1.a){
                manip.setPower(1);
                manipPull.setPosition(1);
                manipTop.setPower(1);
            }
            else if (gamepad1.b){
                manip.setPower(-1);
                manipPull.setPosition(-1);
                manipTop.setPower(-1);
            }
            else {
                manip.setPower(0);
                manipPull.setPosition(0);
                manipTop.setPower(0);
            }


            fly.setPower(flyPow);

            flyRPM = (Math.abs(fly.getCurrentPosition()) - oldFly) / getRuntime();

            oldFly = Math.abs(fly.getCurrentPosition());

            if (rpmValCount > POLL_RATE - 1) {
                rpmAvg = 0;
                for (int i = 0; i < rpmVals.length; i++) {
                    rpmAvg += rpmVals[i];
                }

                rpmAvg /= POLL_RATE;
                rpmValCount = 0;
            }


            else {
                rpmVals[rpmValCount] = flyRPM;
                rpmValCount++;
            }


            if (desiredRPM <= 0) {
                flyPow = 0;
            }

            if (rpmAvg - desiredRPM <= -100 && flyPow >= -1) {
                flyPow -= Math.abs(rpmAvg - desiredRPM)/200000;
            }

            else if (rpmAvg - desiredRPM >= 100 && flyPow <= 0) {
                flyPow += Math.abs(rpmAvg - desiredRPM)/200000;
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flypow", flyPow * -1);
//            telemetry.addData("Fly Position", fly.getCurrentPosition());
//            telemetry.addData("oldFly", oldFly);
            telemetry.addData("DesiredRPM", desiredRPM);
            telemetry.addData("FlyRPM", rpmAvg);
            telemetry.update();

            resetStartTime();
            Thread.sleep(10);
            idle();
        }
    }

}

