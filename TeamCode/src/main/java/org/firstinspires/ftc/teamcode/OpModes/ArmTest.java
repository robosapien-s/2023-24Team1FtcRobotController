package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.Arm;
import org.firstinspires.ftc.teamcode.wrappers.ArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
@Disabled
public class ArmTest extends LinearOpMode {
    Arm arm;

    JoystickWrapper joystickWrapper;

    @Override
    public void runOpMode() throws InterruptedException {

        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);

        arm = new Arm(telemetry, hardwareMap, joystickWrapper);//, armPidConstants.Kp, armPidConstants.Ki, armPidConstants.Kd, armPidConstants.maxIntegralSum, armPidConstants.a);

        waitForStart();
        while(!isStopRequested()) {
            telemetry.addData("update","");
            arm.update(gamepad2);
            telemetry.update();
        }


    }
}
