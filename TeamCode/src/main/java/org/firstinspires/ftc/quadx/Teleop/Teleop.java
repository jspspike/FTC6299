
package org.firstinspires.ftc.quadx.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    CRServo topPull;

    double flyPow = 0.0;
    double oldFly = 0.0;
    double flyRPM = 0;
    int rpmValCount = 0;
    double[] rpmVals = new double[POLL_RATE];
    double rpmAvg;
    double desiredRPM = 0;
    double offset = 0;

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
        topPull = hardwareMap.crservo.get("topPull");

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
        topPull.setPower(0);


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



        waitForStart();
        runtime.reset();



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
                fly.setPower(flyPow);
                runtime.reset();
            }

            else if (gamepad2.x) {
                desiredRPM = 1450;
            }

            else if (gamepad2.y) {
                fly.setPower(0);
            }

            if (gamepad2.dpad_left) {
                desiredRPM -= 10;
                flyPow += .01;
                Thread.sleep(200);
            }

            else if (gamepad2.dpad_right) {
                desiredRPM += 10;
                flyPow -= .01;
                Thread.sleep(200);
            }

            if (gamepad1.right_trigger > .5){
                manip.setPower(1);
                topPull.setPower(-1);
                manipTop.setPower(-1);
            }
            else if (gamepad1.left_trigger > .5){
                manip.setPower(-1);
                topPull.setPower(1);
                manipTop.setPower(1);
            }
            else {
                manip.setPower(0);
                topPull.setPower(0);
                manipTop.setPower(0);
            }

            if (gamepad1.left_bumper)
                buttonP.setPosition(1);
            else if (gamepad1.right_bumper)
                buttonP.setPosition(0);
            else
                buttonP.setPosition(.5);



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

//            if (runtime.seconds() > 4) {
//                if (desiredRPM <= 0) {
//                    flyPow = 0;
//                }
//
//                if (rpmAvg - desiredRPM <= -100 && flyPow >= -1) {
//                    flyPow -= Math.abs(rpmAvg - desiredRPM) / 200000;
//                } else if (rpmAvg - desiredRPM >= 100 && flyPow <= 0) {
//                    flyPow += Math.abs(rpmAvg - desiredRPM) / 200000;
//                }
//            }

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

