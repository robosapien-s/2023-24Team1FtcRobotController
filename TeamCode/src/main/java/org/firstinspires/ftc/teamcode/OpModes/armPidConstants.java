package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Config
@Disabled
public class armPidConstants {
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double maxIntegralSum = 0;
    public static double a = 0.8;
}
