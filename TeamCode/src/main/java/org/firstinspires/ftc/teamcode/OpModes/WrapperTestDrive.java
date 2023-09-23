package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.DrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
@TeleOp
public class WrapperTestDrive extends LinearOpMode {
    DrivingWrapper drivingWrapper;
    JoystickWrapper joystickWrapper;
    double speed = 1;
    double rotSpeed = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);  //see JoystickWrapper
        drivingWrapper = new DrivingWrapper(hardwareMap, telemetry); //see DrivingWrapper

        waitForStart();
        while (!isStopRequested()) {
            drivingWrapper.Drive(joystickWrapper, speed, rotSpeed); //Driving code. See DrivingWrapper
        }
    }
}
