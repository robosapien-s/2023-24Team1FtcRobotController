package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class setRed extends LinearOpMode {
    RedOrBlue redOrBlue;
    @Override
    public void runOpMode() throws InterruptedException {
        redOrBlue.setIsNotAuto();
        redOrBlue.setBlue();
        waitForStart();
        redOrBlue.setIsNotAuto();
        redOrBlue.setBlue();
    }
}
