package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class setBlue extends LinearOpMode {
    RedOrBlue redOrBlue;
    @Override
    public void runOpMode() throws InterruptedException {
        redOrBlue.setBlue();
    }
}
