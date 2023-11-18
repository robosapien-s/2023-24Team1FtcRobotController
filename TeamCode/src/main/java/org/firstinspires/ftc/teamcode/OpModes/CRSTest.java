package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class CRSTest extends LinearOpMode {
    JoystickWrapper joystickWrapper;
    CRServo crServo = hardwareMap.crservo.get("crServo");

    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        while (!isStopRequested()) {
            crServo.setPower(-joystickWrapper.gamepad1GetRightStickY());
        }
    }
}
