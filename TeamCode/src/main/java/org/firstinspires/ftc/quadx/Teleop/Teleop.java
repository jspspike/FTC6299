/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.quadx.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="TeleOp", group="Teleop")
public class Teleop extends LinearOpMode {

    DcMotor fly;
    //DcMotor manip;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;

    Servo door;
    Servo buttonP;

    double flyPow = 0.0;
    double oldFly = 0.0;
    double flyRPM = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        fly = hardwareMap.dcMotor.get("fly");
        //manip = hardwareMap.dcMotor.get("manip");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        door = hardwareMap.servo.get("door");
        buttonP = hardwareMap.servo.get("buttonP");

        fly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //manip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        door.setPosition(0);
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
                motorBL.setPower(-gamepad1.left_stick_y);
                motorBR.setPower(gamepad1.right_stick_y);
                motorFL.setPower(-gamepad1.left_stick_y);
                motorFR.setPower(gamepad1.right_stick_y);
            } else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            if (gamepad2.left_bumper)
                door.setPosition(0);
            else if (gamepad2.right_bumper)
                door.setPosition(.6);

            if (gamepad2.a) {
                flyPow = -1;
            }

            else if (gamepad2.b) {
                flyPow = -.5;
            }

            else if (gamepad2.x) {
                flyPow += -.75;
            }

            else if (gamepad2.y) {
                flyPow = 0;
            }

            if (gamepad2.dpad_left) {
                flyPow += .05;
            }

            else if (gamepad2.dpad_right) {
                flyPow -= .05;
            }

            fly.setPower(flyPow);

            flyRPM = (Math.abs(fly.getCurrentPosition()) - oldFly) / getRuntime();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flypow", flyPow * -1);
            telemetry.addData("Fly Position", fly.getCurrentPosition());
            telemetry.addData("oldFly", oldFly);
            telemetry.addData("FlyRPM", flyRPM);
            telemetry.update();

            oldFly = Math.abs(fly.getCurrentPosition());
            resetStartTime();
            Thread.sleep(100);
            idle();
        }
    }

}

