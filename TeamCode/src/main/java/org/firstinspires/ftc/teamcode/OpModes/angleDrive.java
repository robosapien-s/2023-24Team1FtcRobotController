package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.usefulStuff.RevIMUv2;
import org.firstinspires.ftc.teamcode.wrappers.AngleDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
@Disabled
public class angleDrive extends LinearOpMode {
    AngleDrivingWrapper drivingWrapper;
    JoystickWrapper joystickWrapper;

    RevIMUv2 revIMU;
    double speed = 1;
    double rotSpeed = 1;

    double Kp;
    double Ki;
    double Kd;
    double maxIntegralSum;
    double a;


    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);  //see JoystickWrapper
        drivingWrapper = new AngleDrivingWrapper(hardwareMap, telemetry); //see DrivingWrapper
        revIMU = new RevIMUv2(hardwareMap, "imu");

        revIMU.init();
        waitForStart();
        while (!isStopRequested()) {
            drivingWrapper.Drive(revIMU, joystickWrapper, speed, rotSpeed);
        }
    }
}

