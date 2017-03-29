package org.firstinspires.ftc.quadx.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

@TeleOp(name="TeleOp", group="Teleop")

public class Teleop extends MyOpMode {

    DcMotor fly;
    DcMotor manip;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor liftL;
    DcMotor liftR;

    Servo door;
    Servo buttonP;
    CRServo winch;
    Servo liftArm;
    Servo boot;
    Servo hold;

    double flyPow = 0.0;
    double oldFly = 0.0;
    double flyRPM = 0;
    double bootDefault = BOOT_CLOSED;
    int rpmValCount = 0;
    double[] rpmVals = new double[POLL_RATE];
    double rpmAvg;
    boolean active = false;
    boolean holdArm = false;

    boolean liftActive = false;
    int lessenPower = 0;
    boolean wallPower = false;

    double gamepad1_left;
    double gamepad1_right;

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
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");

        door = hardwareMap.servo.get("door");
        buttonP = hardwareMap.servo.get("buttonP");
        winch = hardwareMap.crservo.get("winch");
        liftArm = hardwareMap.servo.get("liftArm");
        boot = hardwareMap.servo.get("boot");
        hold = hardwareMap.servo.get("hold");

        fly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        manip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        door.setPosition(.6);
        buttonP.setPosition(BUTTONP_CENTER);
        boot.setPosition(bootDefault);
        hold.setPosition(HOLD_DISABLED);

        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();


        telemetry.addData("Volatage", startingVoltage);
        flyPow = .64;

        waitForStart();
        runtime.reset();


        resetStartTime();

        while (opModeIsActive()) {

            gamepad1_left = gamepad1.left_stick_y;
            gamepad1_right = gamepad1.right_stick_y;

            if (lessenPower == 2) {
                gamepad1_left *= .25;
                gamepad1_right *= .25;
            }

            else if (lessenPower == 1) {
                gamepad1_left *= .55;
                gamepad1_right *= .55;
            }

            if (liftActive) {
                gamepad1_left *= -1;
                gamepad1_right *= -1;
            }

            if (wallPower && liftActive) {
                gamepad1_left *= .4;
                gamepad1_right *= .7;
            }

            else if (wallPower) {
                gamepad1_left *= .7;
                gamepad1_right *= .4;
            }

            if ((Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05) && liftActive) {
                motorBL.setPower(gamepad1_right);
                motorBR.setPower(-gamepad1_left);
                motorFL.setPower(gamepad1_right);
                motorFR.setPower(-gamepad1_left);
            }

            else if ((Math.abs(gamepad1.left_stick_y) > .05 || Math.abs(gamepad1.right_stick_y) > .05)) {
                motorBL.setPower(gamepad1_left);
                motorBR.setPower(-gamepad1_right);
                motorFL.setPower(gamepad1_left);
                motorFR.setPower(-gamepad1_right);
            }

            else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            if (gamepad1.x && (lessenPower == 0 || lessenPower == 1) && runtime.milliseconds() > 350) {
                lessenPower = 2;
                runtime.reset();
            }

            else if (gamepad1.b && (lessenPower == 0 || lessenPower == 2) && runtime.milliseconds() > 350) {
                lessenPower = 1;
                runtime.reset();
            }

            else if ((gamepad1.x  || gamepad1.b) && (lessenPower == 1 || lessenPower == 2) && runtime.milliseconds() > 350) {
                lessenPower= 0;
                runtime.reset();
            }

            if (gamepad1.a && !liftActive && runtime.milliseconds() > 350) {
                liftActive = true;
                runtime.reset();
            } else if (gamepad1.a && liftActive && runtime.milliseconds() > 350) {
                liftActive = false;
                runtime.reset();
            }

            if (gamepad1.b && !liftActive && runtime.milliseconds() > 350) {
                liftActive = true;
                runtime.reset();
            } else if (gamepad1.b && liftActive && runtime.milliseconds() > 350) {
                liftActive = false;
                runtime.reset();
            }

            if (gamepad1.dpad_left) {
                wallPower = false;
            }

            else if (gamepad1.dpad_right) {
                wallPower = true;
            }

            if (gamepad1.y) {
                liftActive = false;
                lessenPower = 0;
                wallPower = false;
            }

            if (gamepad2.right_trigger > .5) {
                winch.setPower(-1.0);
                delay(400);
                hold.setPosition(HOLD_HOLD);
            } else if (gamepad2.left_trigger >.5) {
                winch.setPower(1.0);
                hold.setPosition(HOLD_DISABLED);
            } else {
                winch.setPower(0.0);
            }


            if (gamepad2.left_bumper)
                door.setPosition(.2);
            else if (gamepad2.right_bumper)
                door.setPosition(.6);


            if (gamepad2.a) {
                bootDefault = BOOT_CLOSED;
            }

            else if (gamepad2.x) {
                bootDefault = BOOT_HOLD;
            }

            if (gamepad2.b) {
                active = true;
            }

            else if (gamepad2.y) {
                active = false;
            }

            if (gamepad2.dpad_left) {
                flyPow -= .01;
                Thread.sleep(150);
            }

            else if (gamepad2.dpad_right) {
                flyPow += .01;
                Thread.sleep(150);
            }

            if (gamepad2.dpad_up) {
                boot.setPosition(1.0);
            }

            else {
                boot.setPosition(bootDefault);
            }

            if (gamepad2.right_stick_button && !holdArm && runtime.milliseconds() > 350) {
                holdArm = true;
                runtime.reset();
            }

            else if (gamepad2.right_stick_button && holdArm && runtime.milliseconds() > 350) {
                holdArm = false;
                runtime.reset();
            }

            if (holdArm) {
                liftArm.setPosition(.2);
            }

            else if (gamepad2.dpad_down) {
                liftArm.setPosition(1.0);
            }
            else {
                liftArm.setPosition(0);
            }

            if (gamepad1.right_trigger > .5){
                manip.setPower(-1);
            }
            else if (gamepad1.left_trigger > .5){
                manip.setPower(1);
            }
            else {
                manip.setPower(0);
            }

            if (gamepad1.left_bumper)
                buttonP.setPosition(BUTTONP_LEFT);
            else if (gamepad1.right_bumper)
                buttonP.setPosition(BUTTONP_RIGHT);
            else
                buttonP.setPosition(BUTTONP_CENTER);

            if (Math.abs(gamepad2.left_stick_y) > .05) {
                liftL.setPower(-gamepad2.left_stick_y);
                liftR.setPower(-gamepad2.left_stick_y);
            }

            else {
                liftL.setPower(0);
                liftR.setPower(0);
            }

            if (gamepad2.back)
                fly.setPower(-1);
            else if (active)
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

