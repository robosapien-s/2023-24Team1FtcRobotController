package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUReader extends LinearOpMode {

    IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        waitForStart();
        while (!isStopRequested()){

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Pitch", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
