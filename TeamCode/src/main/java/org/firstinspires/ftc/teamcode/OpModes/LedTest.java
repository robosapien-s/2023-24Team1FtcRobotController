package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@TeleOp(name = "LedTest")
public class LedTest extends LinearOpMode {




    int currIndex = 0;
    ArrayList<DigitalChannel> leds = new ArrayList<DigitalChannel>();

    DigitalChannel configureLed(String configName) {
        DigitalChannel led = hardwareMap.get(DigitalChannel.class, configName);
        led.setMode(DigitalChannel.Mode.OUTPUT);
        return led;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);


        DigitalChannel led = hardwareMap.get(DigitalChannel.class, "led0");
        led.setMode(DigitalChannel.Mode.OUTPUT);
        led.setState(false);

        DigitalChannel led2 = hardwareMap.get(DigitalChannel.class, "led1");
        led2.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setState(false);



        /*

        leds.add(configureLed("led0"));
        leds.add(configureLed("led1"));
        leds.add(configureLed("led2"));
        leds.add(configureLed("led3"));
        leds.add(configureLed("led4"));
        leds.add(configureLed("led5"));
        leds.add(configureLed("led6"));
        leds.add(configureLed("led7"));


        HashMap<Integer, ArrayList<DigitalChannel>> ledMap = new HashMap<Integer, ArrayList<DigitalChannel>>();
        ledMap.put(0, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(0))));
        ledMap.put(1, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(0), leds.get(1))));

        ledMap.put(2, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(1))));
        ledMap.put(3, new ArrayList<DigitalChannel>(Arrays.asList(leds.get(1), leds.get(2))));

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
        */
        waitForStart();




        if (opModeIsActive()) {
            while (opModeIsActive()) {



                /*
                if(joystickWrapper.gamepad1GetRightBumperDown()) {
                    currIndex++;
                    if(currIndex > 12) {
                        currIndex = 0;
                    }
                }

                if(joystickWrapper.gamepad1GetLeftBumperDown()) {
                    currIndex--;
                    if(currIndex < 0) {
                        currIndex = leds.size()-1;
                    }
                }


                telemetry.addData("index: ", currIndex);


                for(int i = 0; i < leds.size(); i++) {

                    leds.get(i).setState(false);

                }


                for (DigitalChannel led: ledMap.get(currIndex)) {
                    led.setState(true);
                }

                telemetry.update();
*/



            }
        }

    }
}
