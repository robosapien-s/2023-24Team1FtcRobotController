package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;
import java.util.List;
@Disabled
public class ServoProgrammer {

    int servoCounter = 0;
    ArrayList<Servo> servoArray = new ArrayList<Servo>();
    ArrayList<String> servoStringArray = new ArrayList<String>();

    public ServoProgrammer(){

    }

    public void AddServos(Servo inServo, String servoName){
        servoArray.add(inServo);
        servoStringArray.add(servoName);
    }

    public void Update(JoystickWrapper joystickWrapper, Telemetry telemetry){
        if(joystickWrapper.gamepad1GetDRight()) {
            servoCounter++;
            if(servoCounter >= servoArray.size()) {
                servoCounter = servoArray.size()-1;
            }

            //servoArray.get(servoCounter).setPosition(.5);
        }
        telemetry.addData("Current Servo: ",servoStringArray.get(servoCounter));
        telemetry.addData("position",  servoArray.get(servoCounter).getPosition());
        if(joystickWrapper.gamepad1GetDLeft()) {
            servoCounter--;
            if(servoCounter < 0) {
                servoCounter = 0;
            }

            //servoArray.get(servoCounter).setPosition(.5);
        }

        if(joystickWrapper.gamepad1GetRightBumperDown()) {
            servoArray.get(servoCounter).setPosition(servoArray.get(servoCounter).getPosition()+.05);

            telemetry.addData("position",  servoArray.get(servoCounter).getPosition());
            telemetry.update();
        }

        if(joystickWrapper.gamepad1GetLeftBumperDown()) {
            servoArray.get(servoCounter).setPosition(servoArray.get(servoCounter).getPosition()-.05);
            telemetry.addData("position",  servoArray.get(servoCounter).getPosition());
            telemetry.update();
        }
        telemetry.update();
    }

}
