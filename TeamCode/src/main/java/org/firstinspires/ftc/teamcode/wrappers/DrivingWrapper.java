package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DrivingWrapper {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    public DrivingWrapper(HardwareMap inHardwareMap, Telemetry inTelemetry) {
        hardwareMap = inHardwareMap;
        telemetry = inTelemetry;
        //0
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fL");
        //1
        motorFrontRight = (DcMotorEx)hardwareMap.dcMotor.get("fR");
        //2
        motorBackLeft = (DcMotorEx)hardwareMap.dcMotor.get("bL");
        //3
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("bR");
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE); //setting the right side motors to reverse so they go the right directiond
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double FrontLeftPower(double denominator, double y, double x, double rx) {
        double frontLeftPower = (y + x + rx) / denominator;
        return frontLeftPower;
    }

    public static double BackLeftPower(double denominator, double y, double x, double rx) {
        double backLeftPower = (y - x + rx) / denominator;
        return backLeftPower;
    }

    public static double FrontRightPower(double denominator, double y, double x, double rx) {
        double frontRightPower = (y - x - rx) / denominator;
        return frontRightPower;
    }

    public static double BackRightPower(double denominator, double y, double x, double rx) {
        double backRightPower = (y + x - rx) / denominator;
        return backRightPower;
    }

    public void Drive(JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
        double y = -joystickWrapper.gamepad1GetLeftStickY(); // Remember, this is reversed! | Defining the y variable
        double x = joystickWrapper.gamepad1GetLeftStickX() * 1.1; // Counteract imperfect strafing | Defining the x variable
        double rx = joystickWrapper.gamepad1GetRightStickX() * rotSpeed; // Defining the rx (right x) variable
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // Defining the denominator variable

        motorFrontLeft.setPower(FrontLeftPower(denominator, y, x, rx/speed) * speed); //setting the power for the motors
        motorBackLeft.setPower(BackLeftPower(denominator, y, x, rx/speed) * speed);
        motorFrontRight.setPower(FrontRightPower(denominator, y, x, rx/speed) * speed);
        motorBackRight.setPower(BackRightPower(denominator, y, x, rx/speed) * speed);
    }
    public double calculateDenominator(double x, double y, double rx) {
        return Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    }


}