package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

@TeleOp
public class InitActuator extends LinearOpMode {

    NeoArmWrapper neoArmWrapper;
    JoystickWrapper joystickWrapper;
    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        neoArmWrapper = new NeoArmWrapper(telemetry, hardwareMap, gamepad1, gamepad2, false);
        waitForStart();
        neoArmWrapper.ResetMotorPositions();
        while(!isStopRequested()) {
            neoArmWrapper.MoveActuatorMotor(InitActuatorPos.actuatorPos);
        }
    }
}
