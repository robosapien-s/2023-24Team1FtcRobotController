package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.OpenCvDetection;

@TeleOp
public class testOpenCVBlue extends LinearOpMode{


    JoystickWrapper joystickWrapper;
    OpenCvDetection OpenCVWrapper;
    int signalInt=0;

    boolean red=false;


    @Override
    public void runOpMode() {
        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        OpenCVWrapper = new OpenCvDetection(telemetry, hardwareMap);
        telemetry.update();
        OpenCVWrapper.initColor(red);
        OpenCVWrapper.init();
        waitForStart();
        while (!isStopRequested()) {
            if (joystickWrapper.gamepad1GetA()) {
                OpenCVWrapper.switchColor();
                //red=!red;
                //OpenCVWrapper.initColor(red);
            }
            //telemetry.addData("Barcode:", OpenCVWrapper.barcodeInt);
            //telemetry.update();
        }

    }
}
