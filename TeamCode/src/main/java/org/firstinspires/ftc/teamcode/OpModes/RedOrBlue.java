package org.firstinspires.ftc.teamcode.OpModes;

public class RedOrBlue {
    public static boolean isRed;

    public static boolean isRedAuto;


    public static void setRed() {
        isRed = true;
    }

    public static void setBlue() {
        isRed = false;
    }

    public static void setBlueAuto() {isRedAuto = false;}

    public static void setRedAuto() {isRedAuto = true;}
}
