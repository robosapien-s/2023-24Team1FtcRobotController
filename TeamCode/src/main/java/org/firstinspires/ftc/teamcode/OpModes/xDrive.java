package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.XDriveWrapper;

@TeleOp
@Disabled
public class xDrive extends LinearOpMode {

    XDriveWrapper xDriveWrapper;
    JoystickWrapper joystickWrapper;

    double speed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        xDriveWrapper = new XDriveWrapper(hardwareMap, telemetry);
        while (!isStopRequested()) {
            xDriveWrapper.Drive(joystickWrapper, speed);
        }
    }
}
