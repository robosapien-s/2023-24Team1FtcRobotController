package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RevIMUv2;

public class PIDDrivingWrapper{



    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;

    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    public PIDDrivingWrapper(HardwareMap inHardwareMap, Telemetry inTelemetry) {
        hardwareMap = inHardwareMap;
        telemetry = inTelemetry;
        //0
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fL");
        //1
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fR");
        //2
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bL");
        //3
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("bR");
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE); //setting the right side motors to reverse so they go the right directiond
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double FrontLeftPower(double denominator, double y, double x, double t) {
        double frontLeftPower = (y + x + t) / denominator;
        return frontLeftPower;
    }

    public static double BackLeftPower(double denominator, double y, double x, double t) {
        double backLeftPower = (y - x + t) / denominator;
        return backLeftPower;
    }

    public static double FrontRightPower(double denominator, double y, double x, double t) {
        double frontRightPower = (y - x - t) / denominator;
        return frontRightPower;
    }

    public static double BackRightPower(double denominator, double y, double x, double t) {
        double backRightPower = (y + x - t) / denominator;
        return backRightPower;
    }

    public void Drive(RevIMUv2 revIMU, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
        double angle = revIMU.getHeading();

        double y = -joystickWrapper.gamepad1GetLeftStickY(); // Remember, this is reversed! | Defining the y variable
        double x = joystickWrapper.gamepad1GetLeftStickX() * 1.1; // Counteract imperfect strafing | Defining the x variable
        double t = -joystickWrapper.gamepad1GetRightStickX() * rotSpeed; // Defining the rx (right x) variable
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1); // Defining the denominator variable

        double xr = x * Math.cos(angle) - y * Math.sin(angle); //x with rotation in account
        double yr = x * Math.sin(angle) + y * Math.cos(angle); //y with rotation in account

        motorFrontLeft.setPower(FrontLeftPower(denominator, yr, xr, t / speed) * speed); //setting the power for the motors
        motorBackLeft.setPower(BackLeftPower(denominator, yr, xr, t / speed) * speed);
        motorFrontRight.setPower(FrontRightPower(denominator, yr, xr, t / speed) * speed);
        motorBackRight.setPower(BackRightPower(denominator, yr, xr, t / speed) * speed);
    }
    public double calculateDenominator(double x, double y, double rx) {
        return Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }

}