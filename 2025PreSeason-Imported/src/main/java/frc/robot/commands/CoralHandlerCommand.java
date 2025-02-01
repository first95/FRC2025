package frc.robot.commands;


import java.util.function.BooleanSupplier;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.L1Arm;
import frc.robot.Constants.L1ArmConstants;
import frc.robot.Constants.L1IntakeConstants;


public class CoralHandlerCommand extends Command {
    
    private final BooleanSupplier L1IntakeInButtonSupplier, L1IntakeOutButtonSupplier, L4IntakeButtonSupplier, HandOffButtonSupplier, ScoreButtonSupplier;
    private final L1Arm L1arm;
    private enum State{
        IDLE, L1_INTAKING, L1_HOLDING, L1_SCORE_POSITIONING, L1_SCORING, 
        POSITIONING_HANDOFF, PERFORMING_HANDOFF, 
        L4_INTAKING, L4_HOLDING, L4_SCORING;


    }

    private boolean L1IntakeInButton, L1IntakeOutButton, L4IntakeButton, HandOffButton, ScoreButton, RollerTrigger;
    private boolean coralInL1 = false; 
    private double IntakeCurrent;
    private double L1IntakeSpeed;
    private State currentState = State.IDLE;
    

        public CoralHandlerCommand(BooleanSupplier L1IntakeInButtonSupplier, BooleanSupplier L1IntakeOutButtonSupplier, BooleanSupplier L4IntakeButtonSupplier, 
                                   BooleanSupplier HandOffButtonSupplier, BooleanSupplier ScoreButtonSupplier, 
                                   L1Arm L1arm){
            
    
    
    
            this.L1IntakeInButtonSupplier = L1IntakeInButtonSupplier;
            this.L1IntakeOutButtonSupplier = L1IntakeOutButtonSupplier;
            this.L4IntakeButtonSupplier = L4IntakeButtonSupplier;

            this.HandOffButtonSupplier = HandOffButtonSupplier;
            this.ScoreButtonSupplier = ScoreButtonSupplier;

            this.L1arm = L1arm;
            addRequirements(L1arm);

    }

   

    public void initalize(){
        currentState = State.IDLE;
    }

    public void execute(){
        // read in the inputs
        if (currentState == null) {currentState = State.IDLE;} 

        L1IntakeInButton = L1IntakeInButtonSupplier.getAsBoolean();
        L1IntakeOutButton = L1IntakeOutButtonSupplier.getAsBoolean();
        L4IntakeButton = L4IntakeButtonSupplier.getAsBoolean();

        HandOffButton = HandOffButtonSupplier.getAsBoolean();
        ScoreButton = ScoreButtonSupplier.getAsBoolean();

        L1arm.runIntake(L1IntakeSpeed);

        coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.HOLDING_CURRENT_THRESHOULD;
        // State Machine 
        switch (currentState) {
            case IDLE:
                // L1 ~90, 
                L1arm.setArmAngle(L1ArmConstants.STOWED);
                
                //L4 stowed 
                

                L1IntakeSpeed = 0;
                // change state?

                if(L1IntakeInButton){
                    currentState = State.L1_INTAKING;
                }

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                }



            break;


            case L1_INTAKING:
                // Move L1 to intaking pos, make sure L4 is stowed
                L1arm.setArmAngle(L1ArmConstants.LOWER_LIMIT);
                // wait for spike in current that means we have coral

                L1IntakeSpeed = L1IntakeInButton ? L1IntakeConstants.INTAKE_SPEED : 0;

                if(coralInL1){
                    currentState = State.L1_HOLDING;
                }
                

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                }

            break;

            case L1_HOLDING:
                // Stow L1 
                L1arm.setArmAngle(L1ArmConstants.STOWED);

                L1IntakeSpeed = L1IntakeConstants.HOLDING_SPEED;

                if(ScoreButton){

                    currentState = State.L1_SCORE_POSITIONING;
                }
                
                if(HandOffButton){
                    currentState = State.POSITIONING_HANDOFF;
                }

            break;


            case L1_SCORE_POSITIONING:
                // Move to scoring position then stop

                L1arm.setArmAngle(L1ArmConstants.SCORING);


                if(L1arm.atGoal() && ScoreButton){
                    currentState = State.L1_SCORING;
                }
                

            break;


            case L1_SCORING:
                // Move the roller forward
                
                IntakeCurrent = L1arm.getIntakeCurrent();
                
                L1arm.runIntake(L1IntakeConstants.SCORE_SPEED);
                // If current drops then return to IDLE
                if(IntakeCurrent > L1ArmConstants.CURRENT_OFFSET + L1arm.getIntakeCurrent()){
                    currentState = State.IDLE;
                }

            break;


            case POSITIONING_HANDOFF:
                L1arm.setArmAngle(L1ArmConstants.HAND_OFF);
                
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
                L1arm.setArmAngle(L1ArmConstants.STOWED);

                if(L1IntakeInButton){
                    currentState = State.IDLE;

                }

                // if holding coral go to L4_HOLDING



            break;


            case L4_HOLDING:

                if(ScoreButton){
                    currentState = State.L4_SCORING;
                }

                // Handoff fail or misclick
                if(L1IntakeInButton){
                    currentState = State.L1_INTAKING;
                }

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                }

            break;


            case L4_SCORING:
                if(L1IntakeInButton){
                    currentState = State.IDLE;
            }

                if(L4IntakeButton){
                    currentState = State.IDLE;
            }



            break;


   }





    }

}