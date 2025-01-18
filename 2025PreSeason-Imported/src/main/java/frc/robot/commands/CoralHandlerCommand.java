package frc.robot.commands;


import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.Command;


public class CoralHandlerCommand extends Command {
    
    private final BooleanSupplier L1IntakeButtonSupplier, L4IntakeButtonSupplier, HandOffButtonSupplier, ScoreButtonSupplier, RollerTriggerSupplier;
    //private final L1Arm L1arm;
    //private final L4Arm L4arm;

        public CoralHandlerCommand(BooleanSupplier L1IntakeButtonSupplier, BooleanSupplier L4IntakeButtonSupplier, 
                                   BooleanSupplier HandOffButtonSupplier, BooleanSupplier ScoreButtonSupplier, 
                                   BooleanSupplier RollerTriggerSupplier){
            
    
    
    
            this.L1IntakeButtonSupplier = L1IntakeButtonSupplier;
            this.L4IntakeButtonSupplier = L4IntakeButtonSupplier;

            this.HandOffButtonSupplier = HandOffButtonSupplier;
            this.ScoreButtonSupplier = ScoreButtonSupplier;

            this.RollerTriggerSupplier = RollerTriggerSupplier;

    }

    public void initalize(){
        currentState = State.IDLE;



    }

    private enum State{
        IDLE, L1_INTAKING, L1_HOLDING, L1_SCORE_POSITIONING, L1_SCORING, 
        POSITIONING_HANDOFF, PERFORMING_HANDOFF, 
        L4_INTAKING, L4_HOLDING, L4_SCORING


    }
    private State currentState;

    private boolean L1IntakeButton, L4IntakeButton, HandOffButton, ScoreButton, RollerTrigger;
    private boolean coralInL1 = false; 

    public void execute(){
        // read in the inputs

        L1IntakeButton = L1IntakeButtonSupplier.getAsBoolean();
        L4IntakeButton = L4IntakeButtonSupplier.getAsBoolean();

        HandOffButton = HandOffButtonSupplier.getAsBoolean();
        ScoreButton = ScoreButtonSupplier.getAsBoolean();


        // State Machine 
        switch (currentState) {
            case IDLE:
                // L1 ~90, L4 stowed 
                


                // change state?

                if(L1IntakeButton){
                    currentState = State.L1_INTAKING;
                }

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                }



            break;


            case L1_INTAKING:
                // Move L1 to intaking pos, make sure L4 is stowed

                // wait for spike in current that means we have coral

                if(coralInL1){
                    currentState = State.L1_HOLDING;

                }
                

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                }


            break;

            case L1_HOLDING:
                // Stow L1 

                if(ScoreButton){

                    currentState = State.L1_SCORE_POSITIONING;
                }
                
                if(HandOffButton){
                    currentState = State.POSITIONING_HANDOFF;
                }

            break;


            case L1_SCORE_POSITIONING:
                // Move to scoring position then stop

                if(RollerTrigger){

                    currentState = State.L1_SCORING;
                }
                

            break;


            case L1_SCORING:
                // Move the roller forward

                // If current drops then return to IDLE

            break;



            case POSITIONING_HANDOFF:
                if(!HandOffButton){
                    currentState = State.PERFORMING_HANDOFF;
            }
                

            break;


            case PERFORMING_HANDOFF:
                // Less resistance then go to L4 holding

                

            break;


            case L4_INTAKING:
                // L4 intaking pos
                // L1 backstop ~95 deg
                if(L1IntakeButton){
                    currentState = State.IDLE;

                }

                // if holding coral go to L4_HOLDING



            break;


            case L4_HOLDING:

                if(ScoreButton){
                    currentState = State.L4_SCORING;
                }

                // Handoff fail or misclick
                if(L1IntakeButton){
                    currentState = State.L1_INTAKING;
                }

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                }

            break;


            case L4_SCORING:
                if(L1IntakeButton){
                    currentState = State.IDLE;
            }

                if(L4IntakeButton){
                    currentState = State.IDLE;
            }



            break;


    }





    }

}