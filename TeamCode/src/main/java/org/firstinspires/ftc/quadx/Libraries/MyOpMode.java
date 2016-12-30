package org.firstinspires.ftc.quadx.Libraries;

import android.util.Log;


import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by jspspike on 1/15/2016.
 */
public abstract class MyOpMode extends LinearOpMode {

    public static final int MOVEMENT_DELAY = 500;

    public static final double DOOR_OPEN = .2;
    public static final double DOOR_CLOSED = .6;

    public static final double BUTTONP_CENTER = .47;
    public static final double BUTTONP_LEFT = 1;
    public static final double BUTTONP_RIGHT = 0;

    public boolean flyWheelRunning = true;

    public static DcMotor motorBL;
    public static DcMotor motorBR;
    public static DcMotor motorFL;
    public static DcMotor motorFR;

    public static DcMotor manip;
    public static DcMotor flywheel;

    public static Servo buttonPusher;
    public static Servo door;

    public static OpticalDistanceSensor floorL;
    public static OpticalDistanceSensor floorR;
    public static ColorSensor beaconL;
    public static ColorSensor beaconR;

    public static BNO055IMU gyro;
    public static BNO055IMU.Parameters gyroParam;

    private static ModernRoboticsI2cRangeSensor ultra;

    public double grayL;
    public double grayR;
    public double turn;
    public double gyroError = 0;

    public double ultraDistance;

    public void hardwareMap() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        floorL = hardwareMap.opticalDistanceSensor.get("floorL");
        floorR = hardwareMap.opticalDistanceSensor.get("floorR");
        beaconL = hardwareMap.colorSensor.get("beaconL");
        beaconR = hardwareMap.colorSensor.get("beaconR");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");

        manip = hardwareMap.dcMotor.get("manip");
        flywheel = hardwareMap.dcMotor.get("fly");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        buttonPusher = hardwareMap.servo.get("buttonP");
        door = hardwareMap.servo.get("door");

        telemetry.addData("Status", "Hardware Mapped");
        telemetry.update();
    }

    public void hardwareMapTroll() {
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");

        floorL = hardwareMap.opticalDistanceSensor.get("floorL");
        floorR = hardwareMap.opticalDistanceSensor.get("floorR");
        beaconL = hardwareMap.colorSensor.get("beaconL");
        beaconR = hardwareMap.colorSensor.get("beaconR");

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        ultra = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");

        telemetry.addData("Status", "Hardware Mapped");
        telemetry.update();

    }

    public void initSensors() {
        telemetry.addData("Sensors", "Initializing...");
        telemetry.update();

        beaconL.setI2cAddress(I2cAddr.create8bit(0x20));
        beaconR.setI2cAddress(I2cAddr.create8bit(0x2a));

        floorL.enableLed(false);
        floorR.enableLed(false);
        beaconL.enableLed(false);
        beaconR.enableLed(false);

        grayL = floorL.getRawLightDetected();
        grayR = floorR.getRawLightDetected();

        Log.w("grayL", "" + grayL);
        Log.w("grayR", "" + grayR);

        ultraDistance = -1;

        gyroParam                     = new BNO055IMU.Parameters();
        gyroParam.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParam.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParam.calibrationDataFile = "AdafruitIMUCalibration.json";
        gyroParam.loggingEnabled      = true;
        gyroParam.loggingTag          = "Gryo";
        gyroParam.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        gyro.initialize(gyroParam);

        telemetry.addData("Sensors", "Initialized");
        telemetry.update();
    }

    public void initServos() {
        buttonPusher.setPosition(BUTTONP_CENTER);

        door.setPosition(DOOR_CLOSED);
    }

    public void delay(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        }
        catch (Exception e) {

        }
    }

    public void setMotors(double left, double right) {
        motorFL.setPower(-left);
        motorBL.setPower(-left);
        motorFR.setPower(right);
        motorBR.setPower(right);
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


        return Math.abs(motorBL.getCurrentPosition());
    }

    public void resetGyro() {
        turn = gyro.getAngularOrientation().firstAngle;
    }

    public int encoderPow() {
        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        if (startingVoltage > 13.9) {
            return 4750;
        }

        if (startingVoltage > 13.8) {
            return 4740;
        }

        else if (startingVoltage > 13.5) {
            return 4720;
        }

        return 4720;

    }

    public double flyPow() {
        double startingVoltage = hardwareMap.voltageSensor.get("Motor Controller 5").getVoltage();

        telemetry.addData("Voltage", startingVoltage);

        if (startingVoltage >= 13.8) {
            return .39;
        }

        else if (startingVoltage >= 13.5) {
            return .4;
        }

        else if (startingVoltage >= 13.3) {
            return .41;
        }

        else if (startingVoltage >= 13) {
            return .42;
        }

        else if (startingVoltage >= 12.7) {
            return .43;
        }

        else if (startingVoltage >= 12.5) {
            return .44;
        }

        else if (startingVoltage >= 12.3) {
            return .46;
        }

        return .47;
    }

    public double getGyroYaw() {

        double turnAbs = Math.abs(turn);
        Orientation angles = gyro.getAngularOrientation();
        if (turnAbs > 270 && Math.abs(angles.firstAngle)< 90)
            return (Math.abs(angles.firstAngle) - (turnAbs - 360));
        else if (turnAbs < 90 && Math.abs(angles.firstAngle) > 270)
            return ((Math.abs(angles.firstAngle) - 360) - turnAbs);
        return (Math.abs(angles.firstAngle) - turnAbs);
    }

    public double getGryoPitch() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.secondAngle;
    }

    public double getGyroRoll() {
        Orientation angles = gyro.getAngularOrientation();
        return angles.thirdAngle;
    }

    public double getUltraDistance() {
        double value = ultra.cmUltrasonic();
        if (value != 255)
            ultraDistance = value;
        return ultraDistance;
    }

    public void setServoSlow(Servo servo, double pos) throws InterruptedException {
        double currentPosition = servo.getPosition();

        if (currentPosition - pos > 0) {
            for (; currentPosition > pos; currentPosition -= .005) {
                servo.setPosition(currentPosition);
                delay(1);
                idle();
            }
        }
        else for (; currentPosition < pos; currentPosition += .005) {
            servo.setPosition(currentPosition);
            delay(1);
            idle();
        }
    }

    public void turnPID(double pow, double deg) throws InterruptedException {turnPID(pow, deg, 5000);}

    public void turnPID(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        delay(MOVEMENT_DELAY);
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

        if (!opModeIsActive())
            return;

        resetGyro();
        delay(MOVEMENT_DELAY);

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

    public void moveToRange(double pow, double deg, int cm) throws InterruptedException {moveToRange(pow, deg, cm, 1.5);}

    public void moveToRange(double pow, double deg, int cm, double threshold) throws InterruptedException {moveToRange(pow, deg, cm, threshold, 4.0);}

    public void moveToRange(double pow, double deg, int cm, double threshold, double red) throws InterruptedException { moveToRange(pow, deg, cm, threshold, red, 15000);}

    public void moveToRange(double pow, double deg, int cm, double threshold, double red, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        ElapsedTime time = new ElapsedTime();
        resetGyro();
        resetEncoders();
        delay(MOVEMENT_DELAY);

        time.reset();

        if (deg > 0) {
            while(deg > getEncoderAverage() && time.milliseconds() < tim) {
                if (getUltraDistance() < cm)
                    setMotors(pow / (red * .5), pow);
                else if (getUltraDistance() > cm)
                    setMotors(pow, pow / (red * .5));

                else {
                    if (getGyroYaw() > threshold)
                        setMotors(pow / red, pow);
                    else if (getGyroYaw() < -threshold)
                        setMotors(pow, pow / red);
                    else
                        setMotors(pow, pow);
                }
                telemetry.addData("Gryo", getGyroYaw());
                telemetry.addData("Ultra", getUltraDistance());
                telemetry.update();
                idle();
            }
        }

        else {
            while(Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim) {
                if (getUltraDistance() < cm)
                    setMotors(pow / (red * 1.5), pow);
                else if (getUltraDistance() > cm)
                    setMotors(pow, pow / (red * 1.5));
                else {
                    if (getGyroYaw() > threshold)
                        setMotors(pow , pow / red);
                    else if (getGyroYaw() < -threshold)
                        setMotors(pow / red, pow);
                    else
                        setMotors(pow, pow);
                }
                telemetry.addData("Gryo", getGyroYaw());
                telemetry.addData("Ultra", getUltraDistance());
                telemetry.update();
                idle();
            }
        }

        stopMotors();
    }

    public void moveTo(double pow, double deg) throws InterruptedException {moveTo(pow, deg, .6);}

    public void moveTo(double pow, double deg, double threshold) throws InterruptedException {moveTo(pow, deg, threshold, 2.2   );}

    public void moveTo(double pow, double deg, double threshold, double red) throws InterruptedException { moveTo(pow, deg, threshold, red, 15000, true);}

    public void moveTo(double pow, double deg, double threshold, double red, int tim, boolean stop) throws InterruptedException {

        if (!opModeIsActive())
            return;

        delay(300);

        ElapsedTime time = new ElapsedTime();


        resetGyro();
        resetEncoders();
        delay(MOVEMENT_DELAY);

        time.reset();

        if (deg > 0) {
            while(deg > getEncoderAverage() && time.milliseconds() < tim && opModeIsActive()) {
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(pow / red, pow);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(pow, pow / red);
                else
                    setMotors(pow, pow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverage());
                telemetry.update();
                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        }

        else {
            while(Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim && opModeIsActive()) {
                if (getGyroYaw() + gyroError > threshold)
                    setMotors(-pow , -pow / red);
                else if (getGyroYaw() + gyroError < -threshold)
                    setMotors(-pow / red, -pow);
                else
                    setMotors(-pow, -pow);

                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.addData("Encoder", getEncoderAverage());
                telemetry.update();
                Log.w("Gyro", "" + getGyroYaw());
                idle();
            }
        }
        if (stop)
            stopMotors();

        gyroError = getGyroYaw() + gyroError;
    }



    public void turnCorr(double pow, double deg) throws InterruptedException {turnCorr(pow, deg, 8000);}

    public void turnCorr(double pow, double deg, int tim) throws InterruptedException {
        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg > 0) {
            while(deg > getGyroYaw() && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) / 80);

                if (newPow < .15)
                    newPow = .15;

                setMotors(newPow, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }
        else {
            while(deg < getGyroYaw() && time.milliseconds() < tim) {
                newPow = pow * (Math.abs(deg - getGyroYaw()) /80);

                if (newPow < .15)
                    newPow = .15;
                setMotors(-newPow, newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (deg < getGyroYaw() && opModeIsActive()) {
                setMotors(-.15, .15);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (deg > getGyroYaw() && opModeIsActive()) {
                setMotors(.15, -.15);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }

    public void arcTurnCorr(double pow, double deg) throws InterruptedException {arcTurnCorr(pow, deg, 6000);}

    public void arcTurnCorr(double pow, double deg, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg > 0) {
            while(deg > getGyroYaw() + gyroError && time.milliseconds() < tim  && opModeIsActive()) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.update();
                idle();
            }
        }
        else {
            while(deg < getGyroYaw() + gyroError && time.milliseconds() < tim && opModeIsActive()) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 70);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Gyro Error", gyroError);
                telemetry.update();
                idle();
            }
        }

        stopMotors();

        if (getGyroYaw() > deg) {
            while (deg < getGyroYaw() + gyroError && opModeIsActive()) {
                if (pow > 0)
                    setMotors(-.14, 0);
                else
                    setMotors(0, .14);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        } else {
            while (deg > getGyroYaw() + gyroError && opModeIsActive()) {
                if (pow > 0)
                    setMotors(0, -.14);
                else
                    setMotors(.14, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }
        stopMotors();
        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }

    public void arcTurn(double pow, double deg) throws InterruptedException {arcTurn(pow, deg, 6000);}

    public void arcTurn(double pow, double deg, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        double newPow;

        ElapsedTime time = new ElapsedTime();

        resetGyro();
        delay(MOVEMENT_DELAY);
        time.reset();

        if (deg + gyroError > 0) {
            while(deg > getGyroYaw() + gyroError && time.milliseconds() < tim  && opModeIsActive()) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) / 80);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(newPow, 0);
                else
                    setMotors(0, -newPow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }
        else {
            while(deg < getGyroYaw() + gyroError && time.milliseconds() < tim && opModeIsActive()) {
                newPow = Math.abs(pow) * (Math.abs(deg - getGyroYaw()) /80);

                if (newPow < .15)
                    newPow = .15;

                if (pow > 0)
                    setMotors(0, newPow);
                else
                    setMotors(-newPow, 0);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.update();
                idle();
            }
        }

        stopMotors();
        delay(100);
        gyroError = getGyroYaw() + gyroError - deg;
    }

    public void untilWhiteRange(double pow, double powWhite, double cm, double deg, int degFail) throws InterruptedException {untilWhiteRange(pow, powWhite, cm, deg, degFail, .6, 1.5, 7000);}

    public void untilWhiteRange(double pow, double powWhite, double cm, double deg, int degFail, double threshold, double reduction, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        resetEncoders();
        resetGyro();
//        grayL = floorL.getRawLightDetected();
//        grayR = floorR.getRawLightDetected();
        delay(300);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (deg > 0) {
            while(deg > getEncoderAverage() && time.milliseconds() < tim && opModeIsActive()) {

                    if (getUltraDistance() > cm && getGyroYaw() < 3) {
                        setMotors(pow, pow / (reduction));
                    }

                    else if (getUltraDistance() < cm && getGyroYaw() > -3) {
                        setMotors(pow / (reduction), pow);
                    }

                    else {

                        if (getGyroYaw() + gyroError > threshold)
                            setMotors(pow / (reduction), pow);
                        else if (getGyroYaw() + gyroError < -threshold)
                            setMotors(pow, pow / (reduction));
                        else
                            setMotors(pow, pow);
                        telemetry.addData("Gyro", getGyroYaw());
                        telemetry.addData("Gyro Error", gyroError);
                        telemetry.addData("Encoder", getEncoderAverage());
                        telemetry.addData("Ultra", getUltraDistance());
                        telemetry.update();
                        Log.w("Gyro", "" + getGyroYaw());
                        idle();
                    }
            }
        }

        else {
            while(Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim && opModeIsActive()) {

                        if (getUltraDistance() > cm && getGyroYaw() > - 3) {
                            setMotors(pow, pow / (reduction));
                        }

                        else if (getUltraDistance() < cm && getGyroYaw() < 3) {
                            setMotors(pow / (reduction), pow);
                        }

                        else {
                            if (getGyroYaw() + gyroError > threshold)
                                setMotors(pow, pow / (reduction));
                            else if (getGyroYaw() + gyroError < -threshold)
                                setMotors(pow / (reduction), pow);
                            else
                                setMotors(pow, pow);
                            telemetry.addData("Gyro", getGyroYaw());
                            telemetry.addData("Gyro Error", gyroError);
                            telemetry.addData("Encoder", getEncoderAverage());
                            telemetry.addData("Ultra", getUltraDistance());
                            telemetry.update();
                            Log.w("Gyro", "" + getGyroYaw());
                            idle();
                        }
                }
            }

        if (pow > 0) {
            while ((floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim  && opModeIsActive()) {
                    if (getUltraDistance() > cm && getGyroYaw() < 3) {
                        setMotors(powWhite, powWhite / reduction);
                    }

                    else if (getUltraDistance() < cm && getGyroYaw() > -3) {
                        setMotors(powWhite / reduction, powWhite);
                    }

                    else {

                        if (getGyroYaw() + gyroError > threshold)
                            setMotors(powWhite / reduction, powWhite);
                        else if (getGyroYaw() + gyroError < -threshold)
                            setMotors(powWhite, powWhite / reduction);
                        else
                            setMotors(powWhite, powWhite);
                    }

                if (degFail < Math.abs(getEncoderAverage())) {
                    untilWhiteRange(-.15, -.15, 14, 0, 3000);
                    break;
                }
                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("FloorL", floorL.getRawLightDetected());
                    telemetry.addData("FloorR", floorR.getRawLightDetected());
                    telemetry.addData("Ultra", getUltraDistance());
                    Log.w("FloorL", "" + floorL.getRawLightDetected());
                    Log.w("FloorR", "" + floorR.getRawLightDetected());
                    telemetry.update();
                    idle();
            }
        }

        else {
            while ((floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim && opModeIsActive()) {
                    if (getUltraDistance() > cm  && getGyroYaw() > -3) {
                        setMotors(powWhite, powWhite / reduction);
                    }

                    else if (getUltraDistance() < cm  && getGyroYaw() < 3) {
                        setMotors(powWhite / reduction, powWhite);
                    }

                    else {
                        if (getGyroYaw() + gyroError > threshold) {
                            setMotors(powWhite, powWhite / reduction);
                        } else if (getGyroYaw() + gyroError < -threshold) {
                            setMotors(powWhite / reduction, powWhite);
                        } else {
                            setMotors(powWhite, powWhite);
                        }
                    }

                if (degFail < Math.abs(getEncoderAverage())) {
                    untilWhiteRange(.15, .15, 14, 0, 3000);
                    break;
                }
                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("FloorL", floorL.getRawLightDetected());
                    telemetry.addData("FloorR", floorR.getRawLightDetected());
                    telemetry.addData("Ultra", getUltraDistance());
                    Log.w("FloorL", "" + floorL.getRawLightDetected());
                    Log.w("FloorR", "" + floorR.getRawLightDetected());
                    telemetry.update();
                    idle();
            }
        }

        gyroError = getGyroYaw() + gyroError;
        stopMotors();
        }

    public void untilWhite(double pow, double powWhite) throws InterruptedException {untilWhite(pow, powWhite, 0, 10000);}

    public void untilWhite(double pow, double powWhite, int deg, int degFail) throws InterruptedException {untilWhite(pow, powWhite, deg, degFail, .6, 1.75, 7000);}

    public void untilWhite(double pow, double powWhite, int deg, int degFail, double threshold, double reduction, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        resetEncoders();
        resetGyro();
//        grayL = floorL.getRawLightDetected();
//        grayR = floorR.getRawLightDetected();
        delay(300);

        ElapsedTime time = new ElapsedTime();
        time.reset();

        if (deg > 0) {
            while(deg > getEncoderAverage() && time.milliseconds() < tim && opModeIsActive()) {

                    if (getGyroYaw() + gyroError > threshold)
                        setMotors(pow / reduction, pow);
                    else if (getGyroYaw() + gyroError < -threshold)
                        setMotors(pow, pow / reduction);
                    else
                        setMotors(pow, pow);
                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("Encoder", getEncoderAverage());
                    telemetry.update();
                    Log.w("Gyro", "" + getGyroYaw());
                    idle();
            }
        }

        else {
            while(Math.abs(deg) > getEncoderAverage() && time.milliseconds() < tim && opModeIsActive()) {

                    if (getGyroYaw() + gyroError > threshold)
                        setMotors(pow, pow / reduction);
                    else if (getGyroYaw() + gyroError < -threshold)
                        setMotors(pow / reduction, pow);
                    else
                        setMotors(pow, pow);

                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("Encoder", getEncoderAverage());
                    telemetry.update();
                    Log.w("Gyro", "" + getGyroYaw());
                    idle();
            }
        }

        if (pow > 0) {
            while ((floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim && opModeIsActive()) {

                    if (getGyroYaw() + gyroError > threshold)
                        setMotors(powWhite / reduction, powWhite);
                    else if (getGyroYaw() + gyroError < -threshold)
                        setMotors(powWhite, powWhite / reduction);
                    else
                        setMotors(powWhite, powWhite);

                if (degFail < Math.abs(getEncoderAverage())) {
                    untilWhite(-.15, -.15, 0, 3000);
                    break;
                }
                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("FloorL", floorL.getRawLightDetected());
                    telemetry.addData("FloorR", floorR.getRawLightDetected());
                    Log.w("FloorL", "" + floorL.getRawLightDetected());
                    Log.w("FloorR", "" + floorR.getRawLightDetected());
                    telemetry.update();
                    idle();
            }
        }

        else {
            while ((floorL.getRawLightDetected() < grayL + .5 && floorR.getRawLightDetected() < grayR + .5) && time.milliseconds() < tim && opModeIsActive()) {

                    if (getGyroYaw() + gyroError > threshold) {
                        setMotors(powWhite, powWhite / reduction);
                    } else if (getGyroYaw() + gyroError < -threshold) {
                        setMotors(powWhite / reduction, powWhite);
                    } else {
                        setMotors(powWhite, powWhite);
                    }

                if (degFail < Math.abs(getEncoderAverage())) {
                    untilWhite(.15, .15, 0, 3000);
                    break;
                }

                    telemetry.addData("Gyro", getGyroYaw());
                    telemetry.addData("Gyro Error", gyroError);
                    telemetry.addData("FloorL", floorL.getRawLightDetected());
                    telemetry.addData("FloorR", floorR.getRawLightDetected());
                    Log.w("FloorL", "" + floorL.getRawLightDetected());
                    Log.w("FloorR", "" + floorR.getRawLightDetected());
                    telemetry.update();
                    idle();
            }
        }

        gyroError = getGyroYaw() + gyroError;
        stopMotors();
    }
    
    public void whiteTurn(double pow, int turns) {
        int count = 0;

        while (count < turns) {
            while(floorR.getRawLightDetected() > grayR - .4) {
                setMotors(0, pow);
            }

            count++;
            if (floorL.getRawLightDetected() < grayL - .4 && floorR.getRawLightDetected() < grayR - .4)
                break;

            while (floorL.getRawLightDetected() > grayL - .4) {
                setMotors(-pow, 0);
            }

            count++;

            if (floorL.getRawLightDetected() < grayL - .4 && floorR.getRawLightDetected() < grayR - .4)
                break;
        }
    }

    public void pressRed() {

        if (!opModeIsActive())
            return;

        delay(300);

        int redLeft = 0;

        redLeft += beaconL.red() - beaconR.red();
        redLeft += beaconR.blue() - beaconL.blue();

        if (redLeft > 0) {
            buttonPusher.setPosition(.45);
            delay(100);
            buttonPusher.setPosition(BUTTONP_LEFT);
            delay(800);
            buttonPusher.setPosition(BUTTONP_CENTER);
        }

        else {
            buttonPusher.setPosition(.55);
            delay(100);
            buttonPusher.setPosition(BUTTONP_RIGHT);
            delay(800);
            buttonPusher.setPosition(BUTTONP_CENTER);

        }
    }

    public void pressBlue() {

        if (!opModeIsActive())
            return;

        delay(300);

        int blueLeft = 0;


        blueLeft += beaconL.blue() - beaconR.blue();
        blueLeft += beaconR.red() - beaconL.red();

        if (blueLeft > 0) {
            buttonPusher.setPosition(.45);
            delay(100);
            buttonPusher.setPosition(BUTTONP_LEFT);
            delay(800);
            buttonPusher.setPosition(BUTTONP_CENTER);

        }

        else {
            buttonPusher.setPosition(.55);
            delay(100);
            buttonPusher.setPosition(BUTTONP_RIGHT);
            delay(800);
            buttonPusher.setPosition(BUTTONP_CENTER);
        }
    }
    


    public void flyWheel(final double desiredSpeed) {
        Runnable flyLoop = new Runnable() {
            @Override
            public void run() {
                delay(300);

                int prevEncoderVal;
                double pow = .65;

                flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                prevEncoderVal = flywheel.getCurrentPosition();

                double speed;
                double error;

                while (flyWheelRunning && opModeIsActive()) {
                    resetStartTime();

                    flywheel.setPower(pow);

                    try {
                        idle();
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    delay(100);

                    speed = (flywheel.getCurrentPosition() - prevEncoderVal) / getRuntime();
                    prevEncoderVal = flywheel.getCurrentPosition();

                    error = desiredSpeed - speed;
                    pow += error * .002;
                }
            }
        };
    }
    public void untilRange(double pow) throws InterruptedException {untilRange(pow, 30, .5, 2.2, 7000);}

    public void untilRange(double pow, double endDistance, double threshold, double reduction, int tim) throws InterruptedException {

        if (!opModeIsActive())
            return;

        resetEncoders();
        delay(1000);

        ElapsedTime time = new ElapsedTime();
        time.reset();

            if (pow > 0) {
            while ((getUltraDistance() < endDistance) && time.milliseconds() < tim  && opModeIsActive()) {
                if (getGyroYaw() > threshold)
                    setMotors(pow / reduction, pow);
                else if (getGyroYaw() < -threshold)
                    setMotors(pow, pow / reduction);
                else
                    setMotors(pow, pow);
                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Ultrasonic", getUltraDistance());
                Log.w("Ultrasonic", "" + getUltraDistance());
                telemetry.update();
                idle();
            }
        }

        else {
            while ((getUltraDistance() > endDistance) && time.milliseconds() < tim && opModeIsActive()) {
                if (getGyroYaw() > threshold) {
                    setMotors(pow, pow / reduction);
                }
                else if (getGyroYaw() < -threshold) {
                    setMotors(pow / reduction, pow);
                }
                else {
                    setMotors(pow, pow);
                }

                telemetry.addData("Gyro", getGyroYaw());
                telemetry.addData("Ultrasonic", getUltraDistance());
                Log.w("Ultrasonic", "" + getUltraDistance());
                telemetry.update();
                idle();
            }
        }
        stopMotors();
    }
}

