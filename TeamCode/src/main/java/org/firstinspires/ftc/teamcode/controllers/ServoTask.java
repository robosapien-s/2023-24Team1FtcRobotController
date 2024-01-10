package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoTask extends RobotTaskTimed {

    Servo _servo;
    double _position;
    String _name;
    private ServoTask(){
        super(0);
    }
    public ServoTask(Servo servo, double position, long inDuration, String name) {
        super(inDuration);
        _servo = servo;
        _position = position;
        _name = name;
    }

    @Override
    public void execute(Telemetry telemetry) {

        if(!hasStarted()) {
            _servo.setPosition(_position);
            telemetry.addData("Servo: " + _name, "Executed");
        }
        super.execute(telemetry);
    }
}
