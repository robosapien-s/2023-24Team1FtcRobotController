package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.OpenCvDetection;
@TeleOp
public class testOpenCV extends LinearOpMode{


    Telemetry telemetry;
    HardwareMap hardwareMap;
    public OpenCvDetection OpenCVWrapper;
    int signalInt=0;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCVWrapper = new OpenCvDetection(telemetry, hardwareMap);
        OpenCVWrapper.init(true);
        waitForStart();
        while (!isStopRequested()) {
            signalInt = OpenCVWrapper.barcodeInt;
            //telemetry.addData("Barcode:", OpenCVWrapper.barcodeInt);
            //telemetry.update();
        }

    }
}
