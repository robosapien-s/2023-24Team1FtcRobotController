package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.ArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class ArmTest extends LinearOpMode {
    ArmWrapper armWrapper;
    JoystickWrapper joystickWrapper;


    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        armWrapper = new ArmWrapper(hardwareMap, telemetry);//, armPidConstants.Kp, armPidConstants.Ki, armPidConstants.Kd, armPidConstants.maxIntegralSum, armPidConstants.a);

        waitForStart();
        while(!isStopRequested()) {
            armWrapper.PPArmMove(joystickWrapper);
        }


    }
}
