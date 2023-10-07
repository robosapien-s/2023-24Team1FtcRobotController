package org.firstinspires.ftc.teamcode.wrappers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RevIMUv2;
import org.firstinspires.ftc.teamcode.util.PIDController;


public class PIDDrivingWrapper {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    PIDController pidFrontLeft;
    PIDController pidFrontRight;
    PIDController pidBackLeft;
    PIDController pidBackRight;


    public PIDDrivingWrapper(HardwareMap inHardwareMap, Telemetry inTelemetry,
                             double Kp, double Ki, double Kd, double maxIntegralSum, double a) {
        hardwareMap = inHardwareMap;
        telemetry = inTelemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fL");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fR");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bL");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("bR");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidFrontLeft = new PIDController(Kp, Ki, Kd, maxIntegralSum, a);
        pidFrontRight = new PIDController(Kp, Ki, Kd, maxIntegralSum, a);
        pidBackLeft = new PIDController(Kp, Ki, Kd, maxIntegralSum, a);
        pidBackRight = new PIDController(Kp, Ki, Kd, maxIntegralSum, a);
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
        ElapsedTime dt = new ElapsedTime();

        double angle = -Math.toRadians(revIMU.getHeading());

        double y = -joystickWrapper.gamepad1GetLeftStickY(); // Remember, this is reversed!
        double x = joystickWrapper.gamepad1GetLeftStickX() * 1.1;
        double t = joystickWrapper.gamepad1GetRightStickX() * rotSpeed;

        double xr = (x * Math.cos(angle)) - (y * Math.sin(angle));
        double yr = (x * Math.sin(angle)) + (y * Math.cos(angle));

        double denominator = Math.max(Math.abs(yr) + Math.abs(xr) + Math.abs(t), 1);


        // Calculate errors for each motor
        double errorFrontLeft = FrontLeftPower(denominator, yr, xr, t / speed) * speed - motorFrontLeft.getPower();
        double errorFrontRight = FrontRightPower(denominator, yr, xr, t / speed) * speed - motorFrontRight.getPower();
        double errorBackLeft = BackLeftPower(denominator, yr, xr, t / speed) * speed - motorBackLeft.getPower();
        double errorBackRight = BackRightPower(denominator, yr, xr, t / speed) * speed - motorBackRight.getPower();

        // Use the PID controllers to adjust motor powers
        double pidFrontLeftPower = pidFrontLeft.calculatewitherror(errorFrontLeft, dt);
        double pidFrontRightPower = pidFrontRight.calculatewitherror(errorFrontRight, dt);
        double pidBackLeftPower = pidBackLeft.calculatewitherror(errorBackLeft, dt);
        double pidBackRightPower = pidBackRight.calculatewitherror(errorBackRight, dt);

        // Apply motor power adjustments
        motorFrontLeft.setPower(motorFrontLeft.getPower() + pidFrontLeftPower);
        motorBackLeft.setPower(motorBackLeft.getPower() + pidBackLeftPower);
        motorFrontRight.setPower(motorFrontRight.getPower() + pidFrontRightPower);
        motorBackRight.setPower(motorBackRight.getPower() + pidBackRightPower);


        telemetry.addData("heading", angle);
        telemetry.addData("errorFrontLeft", errorFrontLeft);
        telemetry.addData("errorFrontRight", errorFrontRight);
        telemetry.addData("errorBackLeft", errorBackLeft);
        telemetry.addData("errorBackRight", errorBackRight);
        telemetry.update();
        dt.reset();
    }
}

