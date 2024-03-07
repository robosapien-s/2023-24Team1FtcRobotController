package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
@Disabled
public class CRSTest extends LinearOpMode {
    JoystickWrapper joystickWrapper;
    CRServo crServo;
    CRServo crServo2;

    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        crServo = hardwareMap.crservo.get("cr1");
        crServo2 = hardwareMap.crservo.get("cr2");

        while (!isStopRequested()) {

            if(joystickWrapper.gamepad1GetLeftBumperRaw()) {
                crServo.setPower(-1);
                crServo2.setPower(1);
            }

            if(joystickWrapper.gamepad1GetRightBumperRaw()) {
                crServo.setPower(1);
                crServo2.setPower(-1);
            }
        }
    }
}
