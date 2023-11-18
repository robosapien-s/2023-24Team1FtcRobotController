package org.firstinspires.ftc.teamcode.wrappers;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class XDriveWrapper {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    public XDriveWrapper(HardwareMap inHardwareMap, Telemetry inTelemetry) {
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

    public void Drive(JoystickWrapper joystickWrapper, double speed) {
        motorFrontLeft.setPower(-joystickWrapper.gamepad1GetLeftStickY()*speed);
        motorFrontRight.setPower(joystickWrapper.gamepad1GetLeftStickX()*speed);
        motorBackLeft.setPower(-joystickWrapper.gamepad1GetLeftStickY()*speed);
        motorBackRight.setPower(joystickWrapper.gamepad1GetLeftStickX()*speed);

    }
}
