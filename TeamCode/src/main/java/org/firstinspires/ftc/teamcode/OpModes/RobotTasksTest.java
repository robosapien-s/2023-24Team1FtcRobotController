package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.lang.reflect.Array;
import java.util.ArrayList;

@TeleOp
public class RobotTasksTest extends LinearOpMode {

    JoystickWrapper joystickWrapper;
    Servo crServoRearLift;
    Servo crServoRearWristVertical;
    Servo crServoRearClaw;

    Servo crServoFrontLift;
    Servo crServoFrontWristVertical;
    Servo crServoFrontWristHorizontal;
    Servo crServoFrontClaw;





    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    IRobotTask bottomPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(crServoRearLift, 1, delay, "rearLift"));
        parallel.add(new ServoTask(crServoRearWristVertical, .4, delay, "rearLift"));
        parallel.add(new ServoTask(crServoRearClaw, .45, delay, "rearLift"));
        return parallel;
    }

    IRobotTask mediumPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(crServoRearLift, .7, delay, "rearLift"));
        parallel.add(new ServoTask(crServoRearWristVertical, .5, delay, "rearLift"));
        parallel.add(new ServoTask(crServoRearClaw, .45, delay, "rearLift"));
        return parallel;
    }

    IRobotTask highPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(crServoRearLift, .6, delay, ""));
        parallel.add(new ServoTask(crServoRearWristVertical, .7, delay, ""));
        parallel.add(new ServoTask(crServoRearClaw, .45, delay, ""));
        return parallel;
    }

    IRobotTask transfer() {

        RobotTaskSeries series = new RobotTaskSeries();

        series.add(new ServoTask(crServoRearClaw, .65, 500, ""));

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(crServoFrontLift, 0, 2000, ""));
        parallel.add(new ServoTask(crServoFrontWristVertical, 0.84, 2000, ""));
        parallel.add(new ServoTask(crServoFrontWristHorizontal, 0.47, 2000, ""));
        parallel.add(new ServoTask(crServoFrontClaw, 0.45, 2000, ""));

        parallel.add(new ServoTask(crServoRearLift, 0, 2000, ""));
        parallel.add(new ServoTask(crServoRearWristVertical, .65, 2000, ""));

        series.add(parallel);

        series.add(new ServoTask(crServoRearClaw, .45, 2000, ""));
        return series;
    }
    @Override
    public void runOpMode() throws InterruptedException {

        crServoRearLift = hardwareMap.servo.get("rearLift");
        crServoRearWristVertical = hardwareMap.servo.get("rearWristVertical");
        crServoRearClaw = hardwareMap.servo.get("rearClaw");

        crServoFrontLift = hardwareMap.servo.get("frontLift");
        crServoFrontWristVertical = hardwareMap.servo.get("frontWristVertical");
        crServoFrontWristHorizontal = hardwareMap.servo.get("frontWristHorizontal");
        crServoFrontClaw = hardwareMap.servo.get("frontClaw");

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        int servoCounter = 0;

        ArrayList<Servo> servoArray = new ArrayList<Servo>();
        servoArray.add(crServoRearLift);
        servoArray.add(crServoRearWristVertical);
        servoArray.add(crServoRearClaw);
        servoArray.add(crServoFrontLift);
        servoArray.add(crServoFrontWristVertical);
        servoArray.add(crServoFrontWristHorizontal);
        servoArray.add(crServoFrontClaw);


        waitForStart();
        while (!isStopRequested()) {


            if(joystickWrapper.gamepad1GetDRight()) {
                servoCounter++;
                if(servoCounter >= servoArray.size()) {
                    servoCounter = servoArray.size()-1;
                }

                servoArray.get(servoCounter).setPosition(.5);
            }

            if(joystickWrapper.gamepad1GetDLeft()) {
                servoCounter--;
                if(servoCounter < 0) {
                    servoCounter = 0;
                }

                servoArray.get(servoCounter).setPosition(.5);
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



            if(joystickWrapper.gamepad1GetX()) {
                tasks.add(highPickup(1000));
            }

            if(joystickWrapper.gamepad1GetB()) {
                tasks.add(mediumPickup(1000));
            }


            if(joystickWrapper.gamepad1GetA()) {
                tasks.add(bottomPickup(1000));
            }

            if(joystickWrapper.gamepad1GetY()) {
                tasks.add(transfer());
            }

            //telemetry.addData("taskSize", tasks.size());

            if(tasks.size()>0) {

                boolean isStarted =  tasks.get(0).hasStarted();
                boolean isRunning =  tasks.get(0).isRunning();
                boolean isComplete =  tasks.get(0).isComplete();

                telemetry.addData("isStarted", isStarted);
                telemetry.addData("isRunning", isRunning);
                telemetry.addData("isComplete", isComplete);

                tasks.get(0).execute(telemetry);




                if(isComplete){
                    tasks.remove(0);
                    telemetry.addData("taskSize completed", tasks.size());
                }


                //telemetry.update();

            }

            //telemetry.update();
        }

    }
}
