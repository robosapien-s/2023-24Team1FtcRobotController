package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.usefulStuff.RevIMUv2;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.PIDDrivingWrapper;

@Config
@Disabled
@TeleOp
public class PIDDrive extends LinearOpMode{
    PIDDrivingWrapper drivingWrapper;
    JoystickWrapper joystickWrapper;

    FtcDashboard dashboard;

    RevIMUv2 revIMU;

    double speed = 1;
    double rotSpeed = 1;


    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double maxIntegralSum = 0;
    public static double a = 0.8;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);  //see JoystickWrapper
        drivingWrapper = new PIDDrivingWrapper(hardwareMap, telemetry, Kp, Ki, Kd, maxIntegralSum, a); //see DrivingWrapper
        revIMU = new RevIMUv2(hardwareMap, "imu");

        revIMU.init();
        waitForStart();
        while (!isStopRequested()) {
            drivingWrapper.Drive(revIMU, joystickWrapper, speed, rotSpeed);
        }
    }
}
