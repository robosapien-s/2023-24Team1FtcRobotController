package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

@TeleOp
@Disabled
public class InitActuator extends LinearOpMode {

    NeoArmWrapper neoArmWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2, false);
        waitForStart();
        neoArmWrapper.ResetMotorPositions();
        neoArmWrapper.MoveActuatorMotor(InitActuatorPos.actuatorPos);
    }
}
