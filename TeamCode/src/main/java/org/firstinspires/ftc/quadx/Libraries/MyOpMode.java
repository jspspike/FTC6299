package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.opmodes.Autonomous.AdafruitIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jspspike on 1/15/2016.
 */
public abstract class MyOpMode extends LinearOpMode {
    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;

    public static DcMotor liftL;
    public static DcMotor liftR;

    public static DcMotor manip;
    public static DcMotor winch;

    public static Servo clawL;
    public static Servo clawR;

    public static Servo basket;
    public static Servo doorL;
    public static Servo doorR;

    public static Servo hook;

    public static Servo rack;

    public static Servo ratchet;

    public static Servo beltL;
    public static Servo beltR;

    public static ColorSensor sensorRGB;
    public static DeviceInterfaceModule dim;

    public static AdafruitIMU gyro;

    public static final double CLAWL_UP = .4;
    public static final double CLAWL_DOWN = .12;
    public static final double CLAWR_UP = .5;
    public static final double CLAWR_DOWN = .93;

    public static final double DOORL_CLOSE = .05;
    public static final double DOORL_OPEN = .59;
    public static final double DOORL_OPEN_2 = .92;
    public static final double DOORR_CLOSE = .925;
    public static final double DOORR_OPEN = .32;
    public static final double DOORR_OPEN_2 = .03;

    public static final double HOOK_RETRACT = 1;
    public static final double HOOK_DEPLOYED = .2;

    public static final double RATCHET_RETRACT = .11;
    public static final double RATCHET_DEPLOYED = 0;

    public static final double BASKET_A = .51;
    public static final double BASKET_B = .75;
    public static final double BASKET_X = .35;

    public static final double BELTL_SPEED_LEFT = 1;
    public static final double BELTR_SPEED_LEFT = 1;
    public static final double BELTL_SPEED_RIGHT = 0;
    public static final double BELTR_SPEED_RIGHT = 0;

    public static int gray;
    public static double gerror;

    private static int encoderDifference = 0;

    volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];

    public void mapObjects() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        winch = hardwareMap.dcMotor.get("winch");

        manip = hardwareMap.dcMotor.get("manip");

        clawL = hardwareMap.servo.get("clawR");
        clawR = hardwareMap.servo.get("clawL");

        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");

        basket = hardwareMap.servo.get("basket");
        doorL = hardwareMap.servo.get("doorL");
        doorR = hardwareMap.servo.get("doorR");

        hook = hardwareMap.servo.get("hook");

        rack = hardwareMap.servo.get("rack");
        ratchet = hardwareMap.servo.get("ratchet");

        beltL = hardwareMap.servo.get("beltL");
        beltR = hardwareMap.servo.get("beltR");

        dim = hardwareMap.deviceInterfaceModule.get("dim");
        sensorRGB = hardwareMap.colorSensor.get("mr");
    }

    public void mapObjectsTroll() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
    }

    public void initSensors() {
        try {

            hardwareMap.logDevices();

            sensorRGB.enableLed(true);
            waitOneFullHardwareCycle();

            gray = sensorRGB.alpha();

            gyro = new AdafruitIMU(hardwareMap, "hydro", (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2), (byte) AdafruitIMU.OPERATION_MODE_IMU);

            telemetry.addData("Sensors", "Configured");
            waitOneFullHardwareCycle();
        } catch (Exception e) {
            telemetry.addData("Sensors", "Failed");
        }
    }

    public void initServos() {
        clawL.setPosition(CLAWL_UP);
        clawR.setPosition(CLAWR_UP);
        basket.setPosition(BASKET_A);
        doorL.setPosition(DOORL_CLOSE);
        doorR.setPosition(DOORR_CLOSE);

        hook.setPosition(.5);

        ratchet.setPosition(RATCHET_DEPLOYED);
        rack.setPosition(.5);

        beltL.setPosition(.5);
        beltR.setPosition(.5);
    }

    public void setDoorL(int open) {
        if (open == 0) {
            doorL.setPosition(DOORL_OPEN);
        } else if (open == 1) {
            doorL.setPosition(DOORL_OPEN_2);
        } else {
            doorL.setPosition(DOORL_CLOSE);
        }
    }


    public void setDoorR(int open) {
        if (open == 0) {
            doorR.setPosition(DOORR_OPEN);
        } else if (open == 1) {
            doorR.setPosition(DOORR_OPEN_2);
        } else {
            doorR.setPosition(DOORR_CLOSE);
        }
    }

    public void setClawsDown(boolean down) {
        if (down) {
            clawL.setPosition(CLAWL_DOWN);
            clawR.setPosition(CLAWR_DOWN);
        } else {
            clawL.setPosition(CLAWL_UP);
            clawR.setPosition(CLAWR_UP);
        }
    }

    public void setBasket(char pos) throws InterruptedException {
        if (pos == 'a') {
            setServoSlow(BASKET_A);
        } else if (pos == 'x') {
            setServoSlow(BASKET_X);
        } else if (pos == 'b') {
            setServoSlow(BASKET_B);
        } else {
            setServoSlow(BASKET_A);
        }

    }

    public void setRatchet(boolean active) {
        if (active)
            ratchet.setPosition(RATCHET_DEPLOYED);
        else
            ratchet.setPosition(RATCHET_RETRACT);
    }

    public void setHook(boolean active) {
        if (active)
            hook.setPosition(HOOK_DEPLOYED);
        else
            hook.setPosition(HOOK_RETRACT);
    }

    public void simpleWait(int tim) {
        try {
            Thread.sleep(tim);
            waitOneFullHardwareCycle();
        } catch (Exception e) {
        }
    }

    public int redOrBlue() {
        if (((sensorRGB.red() * 255) / 800) > ((sensorRGB.blue() * 255) / 800)) {
            return 1;
        }
        return 0;
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }

    public void setMotors(double left, double right) {
        if (left > 1) {
            left = 1;
        } else if (left < -1) {
            left = -1;
        } else if (right > 1) {
            right = 1;
        } else if (right < -1) {
            right = -1;
        }

        motorBL.setPower(left);
        motorBR.setPower(-right);
        motorFL.setPower(left);
        motorFR.setPower(-right);
    }

    public void resetEncoders() throws InterruptedException {
//        motorBR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        waitOneFullHardwareCycle();
//        motorBL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        waitOneFullHardwareCycle();
//        motorFL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        waitOneFullHardwareCycle();
//        motorFR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
//        waitOneFullHardwareCycle();
//        motorBR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        motorBL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        motorFL.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        motorFR.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        waitOneFullHardwareCycle();
//
//
//        telemetry.addData("Robot", "Encoders Reset");
//        telemetry.addData("RobotMode1", motorFL.getChannelMode());
//        telemetry.addData("RobotMode2", motorBR.getChannelMode());
        encoderDifference = getEncoderAvg();
    }

    public void resetGyro() throws InterruptedException {
        gyro.startIMU();
        waitOneFullHardwareCycle();
    }

    public int getEncoderAvg() {
        int encoders = 2;

        if (Math.abs(motorBR.getCurrentPosition()) < 3) {
            return Math.abs(motorFL.getCurrentPosition());
        }

        if (Math.abs(motorFL.getCurrentPosition()) < 3) {
            return Math.abs(motorBR.getCurrentPosition());
        }
        Log.e("getEncoderAvg", "Both");
        return (Math.abs(motorBR.getCurrentPosition()) + Math.abs(motorFL.getCurrentPosition())) / encoders;
    }

    public double getGyroYaw() {
        gyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);
        return yawAngle[0] * -1;
    }

    public void turn(double pow, int deg) throws InterruptedException {
        turn(pow, deg, 15000);
    }

    public void setServoSlow(double pos) throws InterruptedException {
        double currentPosition = basket.getPosition();

        if (currentPosition - pos > 0) {
            for (; currentPosition > pos; currentPosition -= .005) {
                basket.setPosition(currentPosition);
                Thread.sleep(1);
                waitOneFullHardwareCycle();
                if (gamepad2.left_stick_y > .07 && gamepad2.left_stick_y > .3) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    beltL.setPosition(BELTL_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y < - .07 && gamepad2.left_stick_y > .3) {
                    liftL.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    beltL.setPosition(BELTL_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y > .07 && gamepad2.right_stick_x < -.3) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    beltL.setPosition(BELTL_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y < - .07 && gamepad2.right_stick_x < -.3) {
                    liftL.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    beltL.setPosition(BELTL_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y < -.07 && gamepad1.dpad_up) {
                    liftL.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    winch.setPower(-1);
                }

                else if (gamepad2.left_stick_y < -.07 && gamepad1.dpad_down) {
                    liftL.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    winch.setPower(1);
                }

                else if (gamepad2.left_stick_y > .07 && gamepad1.dpad_up) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    winch.setPower(-1);
                }

                else if (gamepad2.left_stick_y > .07 && gamepad1.dpad_down) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    winch.setPower(1);
                }

                else if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                    beltL.setPosition(BELTL_SPEED_LEFT);
                    beltR.setPosition(BELTR_SPEED_RIGHT);
                }

                else if (gamepad2.left_stick_button) {
                    beltL.setPosition(BELTL_SPEED_LEFT);
                }

                else if (gamepad2.right_stick_button) {
                    beltR.setPosition(BELTR_SPEED_RIGHT);
                }



                else if (gamepad2.right_stick_x > .3) {
                    beltL.setPosition(BELTL_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    liftL.setPower(0);
                    liftR.setPower(0);
                }

                else if (gamepad2.right_stick_x < -.3) {
                    beltL.setPosition(BELTL_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    liftL.setPower(0);
                    liftR.setPower(0);
                }

                else if (gamepad1.dpad_up) {
                    winch.setPower(-1);
                }

                else if (gamepad1.dpad_down) {
                    winch.setPower(1);
                }

                else if (gamepad2.left_stick_y < -.07) {
                    liftL.setPower(gamepad2.left_stick_y);
                    liftR.setPower(-gamepad2.left_stick_y);
                    beltL.setPosition(.5);
                    beltR.setPosition(.5);
                }

                else if (gamepad2.left_stick_y > .07) {
                    liftL.setPower(gamepad2.left_stick_y * .3);
                    liftR.setPower(-gamepad2.left_stick_y * .3);
                    beltL.setPosition(.5);
                    beltR.setPosition(.5);
                }

                else if (Math.abs(gamepad2.right_stick_y) > .07) {
                    winch.setPower(gamepad2.right_stick_y);
                }

                else if (gamepad2.left_stick_x > .7) {
                    liftR.setPower(0);
                    liftL.setPower(Math.abs(gamepad2.left_stick_x));
                }

                else if (gamepad2.left_stick_x < -.7) {
                    liftR.setPower(Math.abs(gamepad2.left_stick_x) * -1);
                    liftL.setPower(0);
                }

                else {
                    winch.setPower(0);
                    liftL.setPower(0);
                    liftR.setPower(0);
                    beltL.setPosition(.5);
                    beltR.setPosition(.5);
                }
            }
        } else {
            for (; currentPosition < pos; currentPosition += .005) {
                basket.setPosition(currentPosition);
                Thread.sleep(1);
                waitOneFullHardwareCycle();
                if (gamepad2.left_stick_y > .07 && gamepad2.left_stick_y > .3) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    beltL.setPosition(BELTL_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y < - .07 && gamepad2.left_stick_y > .3) {
                    liftL.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    beltL.setPosition(BELTL_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y > .07 && gamepad2.right_stick_x < -.3) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    beltL.setPosition(BELTL_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y < - .07 && gamepad2.right_stick_x < -.3) {
                    liftL.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    beltL.setPosition(BELTL_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                }

                else if (gamepad2.left_stick_y < -.07 && gamepad1.dpad_up) {
                    liftL.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    winch.setPower(-1);
                }

                else if (gamepad2.left_stick_y < -.07 && gamepad1.dpad_down) {
                    liftL.setPower(-Math.pow(gamepad2.left_stick_y, 2));
                    liftR.setPower(Math.pow(gamepad2.left_stick_y, 2));
                    winch.setPower(1);
                }

                else if (gamepad2.left_stick_y > .07 && gamepad1.dpad_up) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    winch.setPower(-1);
                }

                else if (gamepad2.left_stick_y > .07 && gamepad1.dpad_down) {
                    liftL.setPower(gamepad2.left_stick_y * .45);
                    liftR.setPower(-gamepad2.left_stick_y * .45);
                    winch.setPower(1);
                }

                else if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                    beltL.setPosition(BELTL_SPEED_LEFT);
                    beltR.setPosition(BELTR_SPEED_RIGHT);
                }

                else if (gamepad2.left_stick_button) {
                    beltL.setPosition(BELTL_SPEED_LEFT);
                }

                else if (gamepad2.right_stick_button) {
                    beltR.setPosition(BELTR_SPEED_RIGHT);
                }



                else if (gamepad2.right_stick_x > .3) {
                    beltL.setPosition(BELTL_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_RIGHT * Math.abs(gamepad2.right_stick_x));
                    liftL.setPower(0);
                    liftR.setPower(0);
                }

                else if (gamepad2.right_stick_x < -.3) {
                    beltL.setPosition(BELTL_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    beltR.setPosition(BELTR_SPEED_LEFT * Math.abs(gamepad2.right_stick_x));
                    liftL.setPower(0);
                    liftR.setPower(0);
                }

                else if (gamepad1.dpad_up) {
                    winch.setPower(-1);
                }

                else if (gamepad1.dpad_down) {
                    winch.setPower(1);
                }

                else if (gamepad2.left_stick_y < -.07) {
                    liftL.setPower(gamepad2.left_stick_y);
                    liftR.setPower(-gamepad2.left_stick_y);
                    beltL.setPosition(.5);
                    beltR.setPosition(.5);
                }

                else if (gamepad2.left_stick_y > .07) {
                    liftL.setPower(gamepad2.left_stick_y * .3);
                    liftR.setPower(-gamepad2.left_stick_y * .3);
                    beltL.setPosition(.5);
                    beltR.setPosition(.5);
                }

                else if (Math.abs(gamepad2.right_stick_y) > .07) {
                    winch.setPower(gamepad2.right_stick_y);
                }

                else if (gamepad2.left_stick_x > .7) {
                    liftR.setPower(0);
                    liftL.setPower(Math.abs(gamepad2.left_stick_x));
                }

                else if (gamepad2.left_stick_x < -.7) {
                    liftR.setPower(Math.abs(gamepad2.left_stick_x) * -1);
                    liftL.setPower(0);
                }

                else {
                    winch.setPower(0);
                    liftL.setPower(0);
                    liftR.setPower(0);
                    beltL.setPosition(.5);
                    beltR.setPosition(.5);
                }
            }
        }
    }

    public void turnCorr(double power, int deg) throws InterruptedException {
        turnCorr(power, deg, 15000);
    }

    public void turnCorr(double power, int deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        simpleWait(750);
        resetGyro();


        double inte = 0;
        double pow;
        double der;
        double error;
        double previousError = deg - getGyroYaw();
        resetStartTime();

        do {
            error = deg - getGyroYaw();
            pow = power * (error) * .0222;
            inte = (inte * .9) + (getRuntime() * (error) * .02);
            der = (error - previousError) / getRuntime() * .002;
            pow += inte/* + der*/;
            if (pow > 1)
                pow = 1;
            else if (pow < -1)
                pow = -1;
            setMotors(-pow, pow);
            telemetry.addData("Robot", "Turn");
            telemetry.addData("Gyro", getGyroYaw());
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("PID", pow);
            telemetry.addData("I", inte);
            resetStartTime();
            previousError = error;
            waitOneFullHardwareCycle();
        } while (Math.abs(pow) > .15 && opModeIsActive());
        stopMotors();
    }

    public void turn(double pow, int deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        simpleWait(750);
        resetGyro();

        if (deg < 0) {
            while (deg < getGyroYaw() && opModeIsActive()) {
                setMotors(pow, -pow);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive()) {
                setMotors(-pow, pow);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (deg < getGyroYaw() && opModeIsActive()) {
                setMotors(pow / 3, -pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive()) {
                setMotors(-pow / 3, pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        }
        stopMotors();
    }

    public void arcTurn(double pow, int deg) throws InterruptedException {
        arcTurn(pow, deg, 4);
    }

    public void arcTurn(double pow, int deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere


        double newPow;
        simpleWait(250);
        resetGyro();

        deg -= gerror;

        ElapsedTime total = new ElapsedTime();
        total.reset();
        total.startTime();

        if (deg < 0) {
            while (deg < getGyroYaw() && opModeIsActive() && total.time() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);
                if (newPow <= .2)
                    newPow = .2;
                setMotors(newPow, 0);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Power", pow);
                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive() && total.time() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);
                if (newPow <= .2)
                    newPow = .2;
                setMotors(0, newPow);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Power", newPow);
                waitOneFullHardwareCycle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (deg < getGyroYaw() && opModeIsActive() && total.time() < tim) {
                setMotors(pow / 3, 0);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive() && total.time() < tim) {
                setMotors(0, pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        }

        Log.e("Error", "" + (deg - getGyroYaw()));
        stopMotors();
    }

    public void slowTurn(double pow, int deg) throws InterruptedException {
        slowTurn(pow, deg, 4);
    }

    public void slowTurn(double pow, int deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere


        double newPow;
        simpleWait(250);
        resetGyro();

        deg -= gerror;

        ElapsedTime total = new ElapsedTime();
        total.reset();
        total.startTime();

        if (deg < 0) {
            while (deg < getGyroYaw() && opModeIsActive() && total.time() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);
                if (newPow <= .2)
                    newPow = .2;
                setMotors(newPow, -newPow);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Power", pow);
                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive() && total.time() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);
                if (newPow <= .2)
                    newPow = .2;
                setMotors(-newPow, newPow);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Power", newPow);
                waitOneFullHardwareCycle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (deg < getGyroYaw() && opModeIsActive() && total.time() < tim) {
                setMotors(pow / 3, -pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive() && total.time() < tim) {
                setMotors(-pow / 3, pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        }

        Log.e("Error", "" + (deg - getGyroYaw()));
        stopMotors();
    }

    public void moveTo(double pow, int deg) throws InterruptedException {
        moveTo(pow, deg, 1.5, 3, 15);
    }

    public void moveTo(double pow, int deg, double threshold, double reduction) throws InterruptedException {
        moveTo(pow, deg, threshold, reduction, 15);
    }

    public void moveTo(double pow, int deg, double threshold, double reduction, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        gerror = 0;
        resetEncoders();
        resetGyro();
        simpleWait(250);

        ElapsedTime total = new ElapsedTime();
        total.reset();
        total.startTime();
        int diff;
        if (pow < 0) {
            while (deg > (diff = Math.abs(getEncoderAvg() - encoderDifference)) &&/* opModeIsActive() &&*/ total.time() < tim) {


                if (getGyroYaw() > threshold) {
                    setMotors(pow / reduction, pow);
                } else if (getGyroYaw() < -threshold) {
                    setMotors(pow, pow / reduction);
                } else {
                    setMotors(pow, pow);
                }

                Log.e("Encoder AVG", "" + diff);
                Log.e("Left", "" + motorFL.getCurrentPosition());
                Log.e("Right", "" + motorFR.getCurrentPosition());
                telemetry.addData("Robot", "Move");
                telemetry.addData("Encoder AVG", Math.abs(getEncoderAvg() - encoderDifference));
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorBR.getCurrentPosition());
                telemetry.addData("Gyro", getGyroYaw());

                waitOneFullHardwareCycle();
            }
        } else {
            while (deg > (diff = Math.abs(getEncoderAvg() - encoderDifference)) /*&& opModeIsActive()*/ && total.time() < tim) {
                if (getGyroYaw() < -threshold) {
                    setMotors(pow / reduction, pow);
                } else if (getGyroYaw() > threshold) {
                    setMotors(pow, pow / reduction);
                } else {
                    setMotors(pow, pow);
                }

                Log.e("Encoder AVG", "" + diff);
                Log.e("Left", "" + motorFL.getCurrentPosition());
                Log.e("Right", "" + motorBR.getCurrentPosition());
                telemetry.addData("Robot", "Move");
                telemetry.addData("Encoder AVG", Math.abs(getEncoderAvg() - encoderDifference));
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorBR.getCurrentPosition());
                telemetry.addData("Gyro", getGyroYaw());

                waitOneFullHardwareCycle();
            }
        }

        gerror = getGyroYaw();
        Log.e("Error", "" + (getGyroYaw()));
        stopMotors();
    }

    public void simpleMoveTo(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        //resetEncoders();

        simpleWait(500);
        //long startTime =  System.currentTimeMillis();

        while (deg > getEncoderAvg() - encoderDifference/* && (System.currentTimeMillis() - startTime) < tim*/ && opModeIsActive()) {
            waitOneFullHardwareCycle();
            setMotors(pow, pow);
            waitOneFullHardwareCycle();
            telemetry.addData("Current", "Move");
            telemetry.addData("Average", getEncoderAvg() - encoderDifference);
            telemetry.addData("FL Encoder", motorFL.getCurrentPosition());
            telemetry.addData("FR Encoder", motorFR.getCurrentPosition());
            telemetry.addData("BL Encoder", motorBL.getCurrentPosition());
            telemetry.addData("BR Encoder", motorBR.getCurrentPosition());
        }

        stopMotors();
    }


    public void untilWhite(double pow) throws InterruptedException {
        untilWhite(pow, 1.5, 3, 5);
    }

    public void untilWhite(double pow, double threshold, double reduction, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        resetEncoders();
        resetGyro();
        simpleWait(250);

        ElapsedTime total = new ElapsedTime();
        total.reset();
        total.startTime();
        if (pow < 0) {
            while (sensorRGB.alpha() < gray + 25 &&/* opModeIsActive() &&*/ total.time() < tim) {


                if (getGyroYaw() > threshold) {
                    setMotors(pow / reduction, pow);
                } else if (getGyroYaw() < -threshold) {
                    setMotors(pow, pow / reduction);
                } else {
                    setMotors(pow, pow);
                }

                telemetry.addData("Robot", "White");
                telemetry.addData("Color", sensorRGB.alpha());
                telemetry.addData("Encoder AVG", Math.abs(getEncoderAvg() - encoderDifference));
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorBR.getCurrentPosition());
                telemetry.addData("Gyro", getGyroYaw());

                waitOneFullHardwareCycle();
            }
        } else {
            while (sensorRGB.alpha() < gray + 25 /*&& opModeIsActive()*/ && total.time() < tim) {
                if (getGyroYaw() < -threshold) {
                    setMotors(pow / reduction, pow);
                } else if (getGyroYaw() > threshold) {
                    setMotors(pow, pow / reduction);
                } else {
                    setMotors(pow, pow);
                }

                telemetry.addData("Robot", "Move");
                telemetry.addData("Color", sensorRGB.alpha());
                telemetry.addData("Encoder AVG", Math.abs(getEncoderAvg() - encoderDifference));
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorBR.getCurrentPosition());
                telemetry.addData("Gyro", getGyroYaw());

                waitOneFullHardwareCycle();
            }
        }
        stopMotors();
    }

    public void followWhite(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        resetEncoders();
        resetGyro();
        simpleWait(750);


        ElapsedTime total = new ElapsedTime();
        total.reset();
        total.startTime();
        if (pow < 0) {
            while (deg > getEncoderAvg() - encoderDifference && opModeIsActive() && total.time() < tim) {
                if (((sensorRGB.red() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && getGyroYaw() > 0) {
                    setMotors(pow, pow * .75);
                } else if (((sensorRGB.red() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && getGyroYaw() < 0) {
                    setMotors(pow * .75, pow);
                } else {
                    setMotors(pow, pow);
                }

                telemetry.addData("Robot", "Move");
                telemetry.addData("Encoder AVG", getEncoderAvg() - encoderDifference);
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorBR.getCurrentPosition());
                telemetry.addData("Gyro", getGyroYaw());

                waitOneFullHardwareCycle();
            }
        } else {
            if (deg > getEncoderAvg() - encoderDifference && ((sensorRGB.red() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && getGyroYaw() > 0) {
                setMotors(pow * .75, pow);
            } else if (((sensorRGB.red() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && ((sensorRGB.blue() * 255) / 800) < 900 && getGyroYaw() < 0) {
                setMotors(pow, pow * .75);
            } else {
                setMotors(pow, pow);
            }

            telemetry.addData("Robot", "Move");
            telemetry.addData("Encoder AVG", getEncoderAvg() - encoderDifference);
            telemetry.addData("Left", motorFL.getCurrentPosition());
            telemetry.addData("Right", motorBR.getCurrentPosition());
            telemetry.addData("Gyro", getGyroYaw());

            waitOneFullHardwareCycle();
        }
    }

    public void simpleTurn(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        resetEncoders();
        resetGyro();
        simpleWait(500);

        if (deg > 0) {
            while (deg > getEncoderAvg() && opModeIsActive()) {
                waitOneFullHardwareCycle();
                setMotors(-pow, pow);
                waitOneFullHardwareCycle();
                telemetry.addData("Current", "Turn");
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorFR.getCurrentPosition());
            }
        } else {
            while (Math.abs(deg) > getEncoderAvg() && opModeIsActive()) {
                waitOneFullHardwareCycle();
                setMotors(pow, -pow);
                waitOneFullHardwareCycle();
                telemetry.addData("Current", "Turn");
                telemetry.addData("Left", motorFL.getCurrentPosition());
                telemetry.addData("Right", motorFR.getCurrentPosition());
            }
        }

        stopMotors();
    }

    public void correct(double pow) throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        if (getGyroYaw() > 0) {
            while (getGyroYaw() > 0 && opModeIsActive()) {
                setMotors(pow / 3, -pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        } else {
            while (getGyroYaw() < 0 && opModeIsActive()) {
                setMotors(-pow / 3, pow / 3);
                telemetry.addData("Robot", "Turn");
                telemetry.addData("Gyro", getGyroYaw());
                waitOneFullHardwareCycle();
            }
        }
        stopMotors();
    }

    public void dumpHook() throws InterruptedException {
        if (!opModeIsActive()) return; // In case we've been stopped elsewhere

        rack.setPosition(.5);
        hook.setPosition(0);
        simpleWait(700);
        moveTo(-.25, 250);
        simpleWait(300);
        hook.setPosition(1);
        simpleWait(820);
        hook.setPosition(.5);
        Log.e("Error", "DONE");
    }
}
