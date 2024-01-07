package org.firstinspires.ftc.teamcode.wrappers;

import java.util.List;

public class StateArmWrapper {

    enum InputStates{
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
    enum PickupHeight{
        PICKUPLOW,
        PICKUPMEDIUM,
        PICKUPHIGH
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

    }

    BackArmState backArmState;
    PickupHeight pickupHeight;
    FrontArmState frontArmState;

    List<InputStates> inputBuffer;
    InputStates currentInput;

    public void Update(){
        switch (backArmState){

            case PICKUP:
                if(currentInput != null){
                    switch (currentInput){
                        case TRANSFER:
                            if(frontArmState==FrontArmState.TRANSFER){
                                backArmState = backArmState.MOVINGTRANSFER;
                            }else {
                                backArmState = backArmState.MOVINGWAITINGTRANSFER;
                            }
                        case BackLOWPICKUP:
                            //Go to pickup
                        case BackHIGHPICKUP:
                            //go to pickup
                        case BackMEDIUMPICKUP:
                            //go to pickup
                        default:
                            //do nothing
                            break;
                    }
                }
            case MOVINGTRANSFER:
                //check if servo pos is at wanted location until whenever
                //maybe do some pid

            case MOVINGPICKUP:
                //Do nothing
            case TRANSFER:

            case WAITINGTRANSFER:
                if(frontArmState == FrontArmState.TRANSFER){
                    backArmState = BackArmState.MOVINGTRANSFER;
                }
            case MOVINGWAITINGTRANSFER:
                //do the same as moving transfer but move until right above the transfer place


        }
        switch (frontArmState){
            case TRANSFER:
                if(backArmState == BackArmState.TRANSFER){

                }
            case DROP:
                switch (currentInput){
                    case TRANSFER:
                        frontArmState = FrontArmState.MOVINGTRANSFER;
                    case BackLOWPICKUP:
                        //Go to pickup
                    case BackHIGHPICKUP:
                        //go to pickup
                    case BackMEDIUMPICKUP:
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
                        //
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

    }


}
