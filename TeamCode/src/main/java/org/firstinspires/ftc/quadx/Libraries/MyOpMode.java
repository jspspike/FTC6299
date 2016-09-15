package org.firstinspires.ftc.quadx.Libraries;

import android.util.Log;


import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by jspspike on 1/15/2016.
 */
public abstract class MyOpMode extends LinearOpMode {

    public static final int MOVEMENT_DELAY = 500;

    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;

    public static DcMotor manip;
    public static DcMotor flywheel;

    public static ColorSensor sensorRGB;

    public static BNO055IMU gyro;
    public static BNO055IMU.Parameters gyroParam;


    public int gray;

    public void hardwareMap() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        sensorRGB = hardwareMap.colorSensor.get("color");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        telemetry.addData("Status", "Hardware Mapped");
    }

    public void initSensors() {
        sensorRGB.enableLed(true);
        gray = sensorRGB.alpha();

        gyroParam = new BNO055IMU.Parameters();
        gyroParam.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParam.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationData = gyro.readCalibrationData();
        gyroParam.loggingEnabled      = true;
        gyroParam.loggingTag          = "Gryo";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(gyroParam);
    }

    public void initServos() {

    }

    public void setMotors(double left, double right) {
        motorFL.setPower(left);
        motorBL.setPower(left);
        motorFR.setPower(-right);
        motorBR.setPower(-right);
    }

    public void stopMotors() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
    }

    public void resetEncoders() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getEncoderAverage() {
        int encoders = 0;
        int value = 0;

        if (Math.abs(motorFL.getCurrentPosition()) < 2) {
            value += Math.abs(motorFL.getCurrentPosition());
            encoders++;
        }

        if (Math.abs(motorFR.getCurrentPosition()) < 2) {
            value += Math.abs(motorFR.getCurrentPosition());
            encoders++;
        }

        if (Math.abs(motorBL.getCurrentPosition()) < 2) {
            value += Math.abs(motorBL.getCurrentPosition());
            encoders++;
        }

        if (Math.abs(motorBR.getCurrentPosition()) < 2) {
            value += Math.abs(motorBR.getCurrentPosition());
            encoders++;
        }

        return value / encoders;
    }

    public void resetGyro() {
        gyro.initialize(gyroParam);
    }

    public double getGyroYaw() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.firstAngle;
    }

    public double getGryoPitch() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.secondAngle;
    }

    public double getGyroRoll() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.thirdAngle;
    }

    public void setServoSlow(Servo servo, double pos) throws InterruptedException {
        double currentPosition = servo.getPosition();

        if (currentPosition - pos > 0) {
            for (; currentPosition > pos; currentPosition -= .005) {
                servo.setPosition(currentPosition);
                wait(1);
                idle();
            }
        }
        else {
            for (; currentPosition < pos; currentPosition += .005) {
                servo.setPosition(currentPosition);
                Thread.sleep(1);
                idle();
            }
        }
    }

    public void turnPID(double pow, double deg) throws InterruptedException {turnPID(pow, deg, 5000);}

    public void turnPID(double pow, double deg, int tim) throws InterruptedException {
        wait(MOVEMENT_DELAY);
        resetGyro();

        double inte = 0;
        double power;
        double der;
        double error;
        double previousError = deg - getGyroYaw();

        ElapsedTime time = new ElapsedTime();

        time.reset();
        resetStartTime();

        do {
            error = deg - getGyroYaw();
            power = pow * error * .0222;
            inte = inte + (getRuntime() * error * .02);
            der = (error - previousError) / getRuntime() * .02;

            power += inte + der;

            if (power > 1)
                power = 1;
            else if (power < -1) {
                power = -1;
            }

            setMotors(power, -power);
            resetStartTime();
            previousError = error;
            idle();

        } while (Math.abs(power) > .15 && time.milliseconds() < tim);

        stopMotors();
    }

    public void turn(double pow, double deg) throws InterruptedException {turn(pow, deg, 15000);}

    public void turn(double pow, double deg, int tim) throws InterruptedException {

        resetGyro();
        wait(MOVEMENT_DELAY);

        ElapsedTime time = new ElapsedTime();

        time.reset();

        if (deg > 0) {
            while (deg > getGyroYaw() && time.milliseconds() < tim) {
                setMotors(pow, -pow);
                idle();
            }
        }

        else {
            while (deg < getGyroYaw() && time.milliseconds() < tim) {
                setMotors(-pow, pow);
                idle();
            }
        }

        stopMotors();
    }

    public void moveTo(double pow, double deg) throws InterruptedException {moveTo(pow, deg, 1.5);}

    public void moveTo(double pow, double deg, double threshold) throws InterruptedException {moveTo(pow, deg, threshold, 4.0);}

    public void moveTo(double pow, double deg, double threshold, double red) throws InterruptedException { moveTo(pow, deg, threshold, red, 15000);}

    public void moveTo(double pow, double deg, double threshold, double red, int tim) throws InterruptedException {

        ElapsedTime time = new ElapsedTime();
        resetGyro();
        resetEncoders();
        wait(MOVEMENT_DELAY);

        time.reset();

        if (deg > 0) {
            while(deg > getEncoderAverage() && time.milliseconds() < tim) {
                if (getGyroYaw() > threshold)
                    setMotors(pow / red, pow);
                else if (getGyroYaw() < -threshold)
                    setMotors(pow, pow / red);
                else
                    setMotors(pow, pow);
                idle();
            }
        }

        else {
            while(Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
                if (getGyroYaw() > threshold)
                    setMotors(-pow , -pow / red);
                else if (getGyroYaw() < -threshold)
                    setMotors(-pow / red, -pow);
                else
                    setMotors(pow, pow);
                idle();
            }
        }

        stopMotors();
    }

    public void turnCorr(double pow, double deg) throws InterruptedException {turnCorr(pow, deg, 8000);}

    public void turnCorr (double pow, double deg, int tim) throws InterruptedException {
        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        wait(MOVEMENT_DELAY);
        time.reset();

        if (deg > 0) {
            while(deg > getGyroYaw() && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);

                if (newPow < .2)
                    newPow = .2;

                setMotors(newPow, -newPow);
                idle();
            }
        }
        else {
            while(deg < getGyroYaw() && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) /80);

                if (newPow < .2)
                    newPow = .2;
                setMotors(-newPow, newPow);
                idle();
            }
        }


        stopMotors();
    }
}

