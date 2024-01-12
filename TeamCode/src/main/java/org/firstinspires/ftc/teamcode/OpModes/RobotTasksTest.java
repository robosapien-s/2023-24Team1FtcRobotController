package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

@TeleOp
public class RobotTasksTest extends LinearOpMode {

    JoystickWrapper joystickWrapper;
    Servo ServoRearLift;
    Servo ServoRearWristVertical;
    Servo ServoRearClaw;

    Servo ServoFrontLift;
    Servo ServoFrontWristVertical;
    Servo ServoFrontWristHorizontal;
    Servo ServoFrontClaw;

    int count;



    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    IRobotTask bottomPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, 1, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearWristVertical, .4, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearClaw, .45, delay, "rearLift"));
        return parallel;
    }

    IRobotTask ReadyDropOff(Long delay){
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoFrontLift, .55, delay, "rearLift"));
        parallel.add(new ServoTask(ServoFrontWristVertical, .7, delay, "rearLift"));
        parallel.add(new ServoTask(ServoFrontWristHorizontal, .5, delay, "rearLift"));
        count = 2;
        return parallel;
    }
    IRobotTask CycleDropOff(Long delay){
        RobotTaskParallel parallel = new RobotTaskParallel();
        switch(count){
            case 1: //top
                parallel.add(new ServoTask(ServoFrontLift, .55, delay, "rearLift"));
                parallel.add(new ServoTask(ServoFrontWristVertical, .7, delay, "rearLift"));
                parallel.add(new ServoTask(ServoFrontWristHorizontal, .5, delay, "rearLift"));
                count = 2;
            case 2: //middle
                parallel.add(new ServoTask(ServoFrontLift, .75, delay, "rearLift"));
                parallel.add(new ServoTask(ServoFrontWristVertical, .8, delay, "rearLift"));
                parallel.add(new ServoTask(ServoFrontWristHorizontal, .5, delay, "rearLift"));
                count = 3;
            case 3: //bottom
                parallel.add(new ServoTask(ServoFrontLift, .9, delay, "rearLift"));
                parallel.add(new ServoTask(ServoFrontWristVertical, .85, delay, "rearLift"));
                parallel.add(new ServoTask(ServoFrontWristHorizontal, .5, delay, "rearLift"));
                count = 1;
        }
        return parallel;
    }
    IRobotTask mediumPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, .7, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearWristVertical, .5, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearClaw, .45, delay, "rearLift"));
        return parallel;
    }

    IRobotTask highPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, .6, delay, ""));
        parallel.add(new ServoTask(ServoRearWristVertical, .7, delay, ""));
        parallel.add(new ServoTask(ServoRearClaw, .45, delay, ""));
        return parallel;
    }

    IRobotTask transfer() {

        RobotTaskSeries series = new RobotTaskSeries();

        series.add(new ServoTask(ServoRearClaw, .65, 500, ""));

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoFrontLift, 0, 2000, ""));
        parallel.add(new ServoTask(ServoFrontWristVertical, 0.84, 2000, ""));
        parallel.add(new ServoTask(ServoFrontWristHorizontal, 0.47, 2000, ""));
        parallel.add(new ServoTask(ServoFrontClaw, 0.7, 2000, ""));

        parallel.add(new ServoTask(ServoRearLift, 0, 2000, ""));
        parallel.add(new ServoTask(ServoRearWristVertical, .65, 2000, ""));

        series.add(parallel);

        series.add(new ServoTask(ServoRearClaw, .45, 2000, ""));
        series.add(new ServoTask(ServoFrontClaw, 1, 200,""));
        return series;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ServoRearLift = hardwareMap.servo.get("rearLift");
        ServoRearWristVertical = hardwareMap.servo.get("rearWristVertical");
        ServoRearClaw = hardwareMap.servo.get("rearClaw");

        ServoFrontLift = hardwareMap.servo.get("frontLift");
        ServoFrontWristVertical = hardwareMap.servo.get("frontWristVertical");
        ServoFrontWristHorizontal = hardwareMap.servo.get("frontWristHorizontal");
        ServoFrontClaw = hardwareMap.servo.get("frontClaw");

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        int servoCounter = 0;

        ArrayList<Servo> servoArray = new ArrayList<Servo>();
        ArrayList<String> servoStringArray = new ArrayList<String>();
        servoArray.add(ServoRearLift);
        servoArray.add(ServoRearWristVertical);
        servoArray.add(ServoRearClaw);
        servoArray.add(ServoFrontLift);
        servoArray.add(ServoFrontWristVertical);
        servoArray.add(ServoFrontWristHorizontal);
        servoArray.add(ServoFrontClaw);
        servoStringArray.add("rearLift");
        servoStringArray.add("rearWristVertical");
        servoStringArray.add("rearClaw");
        servoStringArray.add("frontLift");
        servoStringArray.add("frontWristVertical");
        servoStringArray.add("frontWristHorizontal");
        servoStringArray.add("frontClaw");




        waitForStart();
        while (!isStopRequested()) {


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
                //

                //telemetry.update();

            }

            telemetry.update();
        }

    }
}
