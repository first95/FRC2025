package frc.robot.commands;


import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.Command;


public class CoralHandlerCommand extends Command {
    private final BooleanSupplier L1ButtonSupplier, L4ButtonSupplier, IntakeButtonSupplier;
    //private final L1Arm L1arm;
    //private final L4Arm L4arm;

        public CoralHandlerCommand(BooleanSupplier L1ButtonSupplier,
        BooleanSupplier L4ButtonSupplier, BooleanSupplier IntakeButtonSupplier){
            
    
    
    
            this.L1ButtonSupplier = L1ButtonSupplier;
            this.L4ButtonSupplier = L1ButtonSupplier;

            this.IntakeButtonSupplier = IntakeButtonSupplier;

    }

    public void initalize(){
        currentState = State.IDLE;



    }

    private enum State{
        IDLE, L1Scoring, L4Scoring


    }
    private State currentState;

    private boolean L1Button, L4Button, IntakeButton;
    private boolean coralEject; 

    public void execute(){
        // read in the inputs

        L1Button = L1ButtonSupplier.getAsBoolean();
        L4Button = L4ButtonSupplier.getAsBoolean();

        IntakeButton = IntakeButtonSupplier.getAsBoolean();


        // State Machine 
        switch (currentState) {
            case IDLE:
                // L1 and L4 in indexing pos  

                // FINISH THIS ONCE L1 READY

                
                if(IntakeButton){
                    // intake run forward

                }
                



                // change state?

                if(L1Button){
                    currentState = State.L1Scoring;
                }

                if(L4Button){
                    currentState = State.L4Scoring;
                }



            break;


            case L1Scoring:
                
                // L1 arm flip to position
                // intake run backward

                if(coralEject){
                    currentState = State.IDLE;
                }


            break;

            case L4Scoring:
                // move L4 to position

            break;


    }





    }

}