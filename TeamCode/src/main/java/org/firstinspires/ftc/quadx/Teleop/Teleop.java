
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

    double flyPow = 0.0;
    double oldFly = 0.0;
    double flyRPM = 0;
    int rpmValCount = 0;
    double[] rpmVals = new double[POLL_RATE];
    double rpmAvg;
    boolean active = false;

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

        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
                flyPow = -.8;
            }

            else if (gamepad2.b) {
                active = true;
            }

            else if (gamepad2.x) {
                flyPow = -.6;
            }

            else if (gamepad2.y) {
                active = false;
            }

            if (gamepad2.dpad_left) {
                flyPow += .01;
                Thread.sleep(150);
            }

            else if (gamepad2.dpad_right) {
                flyPow -= .01;
                Thread.sleep(150);
            }

            if (gamepad1.right_trigger > .5){
                manip.setPower(1);
                manipTop.setPower(-1);
            }
            else if (gamepad1.left_trigger > .5){
                manip.setPower(-1);
                manipTop.setPower(1);
            }
            else {
                manip.setPower(0);
                manipTop.setPower(0);
            }

            if (gamepad1.left_bumper)
                buttonP.setPosition(1);
            else if (gamepad1.right_bumper)
                buttonP.setPosition(0);
            else
                buttonP.setPosition(.5);


            if (active)
                fly.setPower(flyPow);
            else
                fly.setPower(0);


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



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flypow", flyPow);
//            telemetry.addData("Fly Position", fly.getCurrentPosition());
//            telemetry.addData("oldFly", oldFly);
            telemetry.addData("FlyRPM", rpmAvg);
            telemetry.update();

            resetStartTime();
            Thread.sleep(10);
            idle();
        }
    }

}

