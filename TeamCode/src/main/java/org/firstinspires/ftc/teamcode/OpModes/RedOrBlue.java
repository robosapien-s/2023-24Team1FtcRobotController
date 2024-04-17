package org.firstinspires.ftc.teamcode.OpModes;

public class RedOrBlue {

    public static boolean isAuto = true;
    public static boolean isRed;

    public static boolean isRedAuto;


    public static void setIsAuto() { isAuto = true; }
    public static void setIsNotAuto() { isAuto = false; }
    public static void setRed() {
        isRed = true;
    }

    public static void setBlue() {
        isRed = false;
    }

    public static void setBlueAuto() {isRedAuto = false;}

    public static void setRedAuto() {isRedAuto = true;}
}
