package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Disabled
public class DigitalSensorTest extends LinearOpMode {
    RevTouchSensor digitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() {

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(RevTouchSensor.class, "sensor");

        // set the digital channel to input.


        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (!digitalTouch.isPressed()) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }

            telemetry.update();
        }
    }
}