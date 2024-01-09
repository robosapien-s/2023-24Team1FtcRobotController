package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class StateArmWrapper {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    Servo rearLiftServo;

    Servo rearWristVerticalServo;
    Servo rearClawServo;

    Servo frontLiftServo;
    Servo frontWristVerticalServo;
    Servo frontWristHorizontalServo;
    Servo frontClawServo;

    public enum InputStates{
        FrontDROPLOW, // vertical wrist = .8, horizontal wrist = 0.5 is straight ahead, call it 0˚
        //if straight is 0˚, then 0 servo position = -150˚, 1 servo position = 150˚, should be proportional spectrum so 75˚ should be .75, etc.
        //lift position = 1 when drop
        FrontDROPMEDIUM,
        FrontDROPHIGH,
        FrontLOWPICKUP,
        FrontHIGHPICKUP,
        BackLOWPICKUP, //verticalwrist = .4, lift = 1
        BackMEDIUMPICKUP, //vertical wrist =.5, lift = .7
        BackHIGHPICKUP, //vertical wrist = .7, lift = .6 or .5 depending on height of the stack
        TRANSFER
    }
    enum BackArmState{
        PICKUP,
        MOVINGPICKUP,
        MOVINGWAITINGTRANSFER,
        WAITINGTRANSFER,
        TRANSFER,
        MOVINGTRANSFER
    }

    enum FrontArmState{
        DROP,
        PICKUP,
        TRANSFER,
        MOVINGDROP,
        MOVINGPICKUP,
        MOVINGTRANSFER
    }

    public StateArmWrapper(){
        backArmState = BackArmState.PICKUP;
    }

    BackArmState backArmState;
    FrontArmState frontArmState;

    List<InputStates> inputBuffer;
    InputStates currentInput;

    ElapsedTime frontLiftTimer = new ElapsedTime();
    ElapsedTime backLiftTimer = new ElapsedTime();

    public void Update(){
        switch (backArmState){
            case PICKUP:
                backLiftTimer.reset();
                if(currentInput != null) {
                    switch (currentInput) {
                        case TRANSFER:
                            if (frontArmState == FrontArmState.TRANSFER) {
                                MoveToState(InputStates.TRANSFER);
                                backArmState = backArmState.MOVINGTRANSFER;
                                inputBuffer.remove(0);
                            } else {
                                MoveToState(InputStates.TRANSFER);
                                backArmState = backArmState.MOVINGWAITINGTRANSFER;
                                inputBuffer.remove(0);
                            }
                        case BackLOWPICKUP:
                            MoveToState(InputStates.BackLOWPICKUP);
                            backArmState = BackArmState.MOVINGPICKUP;
                            inputBuffer.remove(0);
                        case BackHIGHPICKUP:
                            MoveToState(InputStates.BackHIGHPICKUP);
                            backArmState = BackArmState.MOVINGPICKUP;
                            inputBuffer.remove(0);
                        case BackMEDIUMPICKUP:
                            MoveToState(InputStates.BackMEDIUMPICKUP);
                            backArmState = BackArmState.MOVINGPICKUP;
                            inputBuffer.remove(0);
                        default:
                            break;
                    }
                }
            case MOVINGTRANSFER:
                if(backLiftTimer.seconds() >= .4){
                    backArmState = BackArmState.TRANSFER;
                }

            case MOVINGPICKUP:
                if(backLiftTimer.seconds() >= .4){
                    backArmState = BackArmState.PICKUP;
                }
            case TRANSFER:
                if(currentInput != null) {
                    switch (currentInput) {
                        case TRANSFER:
                            if (frontArmState == FrontArmState.TRANSFER) {
                                MoveToState(InputStates.TRANSFER);
                                backArmState = backArmState.MOVINGTRANSFER;
                                inputBuffer.remove(0);
                            } else {
                                MoveToState(InputStates.TRANSFER);
                                backArmState = backArmState.MOVINGWAITINGTRANSFER;
                                inputBuffer.remove(0);
                            }
                        default:
                            break;
                    }
                }


            case WAITINGTRANSFER:
                if(frontArmState == FrontArmState.TRANSFER){
                    backArmState = BackArmState.MOVINGTRANSFER;
                }
            case MOVINGWAITINGTRANSFER:
                if(backLiftTimer.seconds() >= .4){
                    backArmState = BackArmState.WAITINGTRANSFER;
                }


        }
        switch (frontArmState){
            case TRANSFER:
                if(backArmState == BackArmState.TRANSFER){

                }
            case DROP:
                switch (currentInput){
                    case TRANSFER:
                        frontArmState = FrontArmState.MOVINGTRANSFER;
                    case FrontDROPHIGH:
                        //Go to pickup
                    case FrontDROPLOW:
                        //go to pickup
                    case FrontDROPMEDIUM:
                        //go to pickup
                    default:
                        //do nothing
                        break;
                }
            case PICKUP:
                switch (currentInput){
                    case TRANSFER:
                        frontArmState = FrontArmState.MOVINGTRANSFER;
                    case FrontHIGHPICKUP:
                        //go to pickup
                    case FrontLOWPICKUP:
                        //go to pickup
                    default:
                        //do nothing
                        break;
                }
            case MOVINGDROP:

            case MOVINGPICKUP:

            case MOVINGTRANSFER:

        }
        telemetry.addData("BackArm",backArmState);
        telemetry.addData("TimeElapsed", backLiftTimer.seconds());
        telemetry.addData("Input", inputBuffer);
        telemetry.update();

    }


    public void MoveToState(FrontArmState state){
        switch (state){
            case DROP:

            case TRANSFER:

            default:

        }
    }
    public void MoveToState(BackArmState state){
        switch(state){
            case TRANSFER:

            case MOVINGPICKUP:

            case PICKUP:

            case MOVINGWAITINGTRANSFER:

            case WAITINGTRANSFER:

            case MOVINGTRANSFER:

        }
    }

    public void MoveToState(InputStates state){
        /*switch (state){
            case BackHIGHPICKUP:
                //vertical wrist = .7, lift = .6 or .5 depending on height of the stack
                rearWristVerticalServo.setPosition(.7);
                rearLiftServo.setPosition(.5);

            case BackLOWPICKUP:
                rearWristVerticalServo.setPosition(.4);
                rearLiftServo.setPosition(1);
                //verticalwrist = .4, lift = 1
            case BackMEDIUMPICKUP:
                rearLiftServo.setPosition(.7);
                rearWristVerticalServo.setPosition(.5);
                //vertical wrist =.5, lift = .7

            case FrontDROPLOW:
                frontWristVerticalServo.setPosition(.8);
                frontWristHorizontalServo.setPosition(getServoRotation());

                // vertical wrist = .8, horizontal wrist = 0.5 is straight ahead, call it 0˚
                //if straight is 0˚, then 0 servo position = -150˚, 1 servo position = 150˚, should be proportional spectrum so 75˚ should be .75, etc.
                //lift position = 1 when drop

            case FrontDROPHIGH:

            case FrontDROPMEDIUM:

            case FrontLOWPICKUP:

            case FrontHIGHPICKUP:

        }*/
    }

    private double getServoRotation() {
        return .5;

    }

    public void AddInput(InputStates input) {
        inputBuffer.add(input);
    }


}
