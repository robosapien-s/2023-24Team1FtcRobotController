package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RevIMUv2;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;

@TeleOp
public class FCDrive extends LinearOpMode {
    FCDrivingWrapper drivingWrapper;
    JoystickWrapper joystickWrapper;

    RevIMUv2 revIMU;
    double speed = 1;
    double rotSpeed = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);  //see JoystickWrapper
        drivingWrapper = new FCDrivingWrapper(hardwareMap, telemetry); //see DrivingWrapper
        revIMU = new RevIMUv2(hardwareMap, "imu");

        revIMU.init();
        waitForStart();
        while (!isStopRequested()) {
            drivingWrapper.Drive(revIMU, joystickWrapper, speed, rotSpeed);
        }
    }
}
