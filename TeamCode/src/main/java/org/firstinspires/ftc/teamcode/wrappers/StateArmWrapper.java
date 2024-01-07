package org.firstinspires.ftc.teamcode.wrappers;

import java.util.List;

public class StateArmWrapper {

    enum InputStates{
        FrontDROPLOW, // vertical wrist = .8, horizontal wrist = 0.5 is straight ahead, call it 0˚
        //if straight is 0˚, then 0 servo position = -150˚, 1 servo position = 150˚, should be proportional spectrum so 75˚ should be .75, etc.
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

                        case BackHIGHPICKUP:
                        case BackMEDIUMPICKUP:
                        default:
                            break;
                    }
                }
            case MOVINGTRANSFER:

            case MOVINGPICKUP:

            case TRANSFER:

            case WAITINGTRANSFER:

            case MOVINGWAITINGTRANSFER:

        }

    }


}
