package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class LedController {


    int currIndex = 0;
    ArrayList<DigitalChannel> leds = new ArrayList<DigitalChannel>();
    HashMap<Integer, ArrayList<DigitalChannel>> ledMap = new HashMap<Integer, ArrayList<DigitalChannel>>();


    public void next() {
        currIndex++;
        if(currIndex > 12) {
            currIndex = 0;
        }

        evaluate();
    }

    public void prev() {
        currIndex--;
        if(currIndex < 0) {
            currIndex = leds.size()-1;
        }

        evaluate();
    }

    public void setCurrentIndex(int inCurrentIndex) {
        currIndex = inCurrentIndex;

        if(currIndex < 0) {
            currIndex = 0;
        } else if (currIndex>12) {
            currIndex = 12;
        }

        evaluate();
    }

    public void evaluate() {

        for(int i = 0; i < leds.size(); i++) {
            leds.get(i).setState(true);
        }

        for (DigitalChannel led: ledMap.get(currIndex)) {
            led.setState(false);
        }
    }
    public LedController(HardwareMap hardwareMap) {


        leds.add(configureLed(hardwareMap, "led0"));
        leds.add(configureLed(hardwareMap, "led1"));
        leds.add(configureLed(hardwareMap, "led2"));
        leds.add(configureLed(hardwareMap, "led3"));
        leds.add(configureLed(hardwareMap, "led4"));
        leds.add(configureLed(hardwareMap, "led5"));
        leds.add(configureLed(hardwareMap, "led6"));
        leds.add(configureLed(hardwareMap, "led7"));

        ledMap.put(0, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(0))));
        ledMap.put(1, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(0), leds.get(1))));
        ledMap.put(2, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(0),leds.get(1),leds.get(2))));
        ledMap.put(3, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(0),leds.get(1),leds.get(2), leds.get(3))));

        ledMap.put(4, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(2))));
        ledMap.put(5, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(2), leds.get(3))));

        ledMap.put(6, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(3))));
        ledMap.put(7, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(3), leds.get(4))));

        ledMap.put(8, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(4))));
        ledMap.put(9, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(4), leds.get(5))));

        ledMap.put(10, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(5))));
        ledMap.put(11, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(5), leds.get(6))));

        ledMap.put(12, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(6))));
        ledMap.put(13, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(6), leds.get(7))));

        evaluate();

    }


    DigitalChannel configureLed(HardwareMap hardwareMap, String configName) {
        DigitalChannel led = hardwareMap.get(DigitalChannel.class, configName);
        led.setMode(DigitalChannel.Mode.OUTPUT);
        return led;
    }
}
