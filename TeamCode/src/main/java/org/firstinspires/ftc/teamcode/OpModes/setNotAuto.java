package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class setNotAuto extends LinearOpMode {
    RedOrBlue redOrBlue;
    @Override
    public void runOpMode() throws InterruptedException {
        redOrBlue.setIsNotAuto();
        waitForStart();
        redOrBlue.setIsNotAuto();
    }
}
//angle = 13.26, slide = 752
//angle = 36.56, slide = 2319