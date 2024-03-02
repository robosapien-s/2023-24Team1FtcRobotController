package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LEDWrapper {

    LED led0;
    LED led1;
    LED led2;
    LED led3;
    LED led4;
    LED led5;
    LED led6;

    public LEDWrapper (HardwareMap hardwareMap, Telemetry telemetry){
        led0 = hardwareMap.get(LED.class, "led0");
        led1 = hardwareMap.get(LED.class, "led1");//Inverted
        led2 = hardwareMap.get(LED.class, "led2");
        led3 = hardwareMap.get(LED.class, "led3");//Inverted
        led4 = hardwareMap.get(LED.class, "led4");
        led5 = hardwareMap.get(LED.class, "led5");//Inverted
        led6 = hardwareMap.get(LED.class, "led6");

    }

    public void UpdateLed(int Location){
        switch (Location){
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:

        }
    }
}
