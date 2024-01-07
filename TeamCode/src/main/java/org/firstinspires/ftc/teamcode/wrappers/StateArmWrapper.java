package org.firstinspires.ftc.teamcode.wrappers;

import java.util.List;

public class StateArmWrapper {

    enum InputStates{
        FrontDROPLOW,
        FrontDROPMEDIUM,
        FrontDROPHIGH,
        FrontLOWPICKUP,
        FrontHIGHPICKUP,
        BackLOWPICKUP,
        BackMEDIUMPICKUP,
        BackHIGHPICKUP,
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
