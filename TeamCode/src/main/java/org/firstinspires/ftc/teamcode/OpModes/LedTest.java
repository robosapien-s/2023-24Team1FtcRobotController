package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp(name = "LedTest")
public class LedTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        waitForStart();

        int index = 0;

        LED led0 = hardwareMap.get(LED.class, "led1");
        LED led1 = hardwareMap.get(LED.class, "led2");

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                if(joystickWrapper.gamepad1GetRightBumperDown()) {
                    index++;
                    if(index > 3) {
                        index = 0;
                    }
                }

                if(joystickWrapper.gamepad1GetLeftBumperDown()) {
                    index--;
                    if(index < 0) {
                        index = 3;
                    }
                }


                if(index == 0) {
                    led0.enableLight(true);
                    led1.enableLight(false);
                } else if(index == 1) {
                    led0.enableLight(true);
                    led1.enableLight(true);
                } else if (index == 2){
                    led0.enableLight(false);
                    led1.enableLight(false);
                } else {
                    led0.enableLight(false);
                    led1.enableLight(true);                }







            }
        }

    }
}
