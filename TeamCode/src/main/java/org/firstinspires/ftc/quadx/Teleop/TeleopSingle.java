package org.firstinspires.ftc.quadx.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.quadx.Libraries.MyOpMode;

@TeleOp(name="TeleOp Single", group="Teleop")

public class TeleopSingle extends MyOpMode {

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
        flyPow = .6;

        waitForStart();
        runtime.reset();


        resetStartTime();

        while (opModeIsActive()) {

            gamepad1_left = gamepad1.left_stick_y;
            gamepad1_right = gamepad1.right_stick_y;

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

            if (gamepad1.left_bumper)
                door.setPosition(.2);
            else if (gamepad1.right_bumper)
                door.setPosition(.6);

            if (gamepad1.b) {
                active = true;
            }

            else if (gamepad1.y) {
                active = false;
            }

            if (gamepad1.dpad_left) {
                flyPow -= .01;
                Thread.sleep(150);
            }

            else if (gamepad1.dpad_right) {
                flyPow += .01;
                Thread.sleep(150);
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

