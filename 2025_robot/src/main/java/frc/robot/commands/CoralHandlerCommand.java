package frc.robot.commands;


import static edu.wpi.first.units.Units.Rotation;

import java.security.spec.ECPublicKeySpec;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.L1Arm;
import frc.robot.subsystems.L4Arm;
import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants.L1ArmConstants;
import frc.robot.Constants.L1IntakeConstants;
import frc.robot.Constants.L4ArmConstants;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.Constants;


public class CoralHandlerCommand extends Command {
    
    private final BooleanSupplier 
        L1IntakeButtonSupplier, 
        L1EjectButtonSupplier, 
        L4IntakeButtonSupplier,
        L4ScoreButtonSupplier, 
        HandOffButtonSupplier, 
        L1ScoreButtonSupplier,
        StowButtonSupplier, 
        L1HumanLoadingSupplier,
        pointToReefButtonSupplier;

    private final L1Arm L1arm;
    private final L4Arm L4arm;
    private final Climber climber;
    private final SwerveBase swerve;
    private final AbsoluteDrive absDrive;
    private enum State{
        IDLE, L1_INTAKING, L1_HOLDING, L1_SCORE_POSITIONING, L1_SCORING, 
        POSITIONING_HANDOFF, PERFORMING_HANDOFF, 
        L4_INTAKING, L4_SCORING, L1_HUMAN_LOADING;
    }

    private boolean 
        L1IntakeButton, 
        L1EjectButton, 
        L4IntakeButton, 
        HandOffButton,
        L1ScoreButton, 
        L4ScoreButton, 
        StowButton, 
        L1HumanLoadButton,
        pointToReefButton;
    
    private boolean 
        autoL4HumanLoadTrigger,
        autoL4ScoreTrigger,
        inAuto;


    private boolean coralInL1 = false;
    private boolean releasedCoral = false; 
    private int releasedCoralCycles = 0;
    private double IntakeCurrent;
    private Rotation2d L4ScoreAngle;
    private int cyclesIntaking;
    private double L1IntakeSpeed;
    private State currentState = State.IDLE;
    

        public CoralHandlerCommand(
            BooleanSupplier L1IntakeButtonSupplier, 
            BooleanSupplier L1EjectButtonSupplier, 
            BooleanSupplier L4IntakeButtonSupplier,
            BooleanSupplier L4ScoreButtonSupplier, 
            BooleanSupplier HandOffButtonSupplier, 
            BooleanSupplier L1ScoreButtonSupplier, 
            BooleanSupplier StowButtonSupplier, 
            BooleanSupplier L1HumanLoadingSupplier,
            BooleanSupplier pointToReefButtonSupplier,
            L1Arm L1arm,
            L4Arm L4arm,
            Climber climber,
            SwerveBase swerve,
            AbsoluteDrive absDrive){
            
    
    
    
            this.L1IntakeButtonSupplier = L1IntakeButtonSupplier;
            this.L1EjectButtonSupplier = L1EjectButtonSupplier;
            this.L1HumanLoadingSupplier = L1HumanLoadingSupplier;
            this.StowButtonSupplier = StowButtonSupplier;
            this.L4IntakeButtonSupplier = L4IntakeButtonSupplier;

            this.HandOffButtonSupplier = HandOffButtonSupplier;
            this.L1ScoreButtonSupplier =L1ScoreButtonSupplier;

            this.L4ScoreButtonSupplier = L4ScoreButtonSupplier;
            this.pointToReefButtonSupplier = pointToReefButtonSupplier;

        
            this.climber = climber;
            this.L1arm = L1arm;
            this.L4arm = L4arm;
            this.swerve = swerve;
            this.absDrive = absDrive;
            addRequirements(L1arm, L4arm, climber);

    }

   

    public void initalize(){
        currentState = State.IDLE;
    }

    public void execute(){
        // read in the inputs
        if (currentState == null) {currentState = State.IDLE;} 

        L1IntakeButton = L1IntakeButtonSupplier.getAsBoolean();
        L1EjectButton = L1EjectButtonSupplier.getAsBoolean();
        L1HumanLoadButton = L1HumanLoadingSupplier.getAsBoolean();
        StowButton = StowButtonSupplier.getAsBoolean();

        L4IntakeButton = L4IntakeButtonSupplier.getAsBoolean();
        L4ScoreButton = L4ScoreButtonSupplier.getAsBoolean();
        
        HandOffButton = HandOffButtonSupplier.getAsBoolean();
        L1ScoreButton=L1ScoreButtonSupplier.getAsBoolean();

        inAuto = SmartDashboard.getBoolean(Constants.Auton.AUTO_ENABLED_KEY, false);
        autoL4HumanLoadTrigger = SmartDashboard.getBoolean(Constants.Auton.L4HUMANLOAD_KEY, false);
        autoL4ScoreTrigger = SmartDashboard.getBoolean(Constants.Auton.L4SCORE_KEY, false);

        pointToReefButton = pointToReefButtonSupplier.getAsBoolean();
        
        if(pointToReefButton){
            //absDrive.setHeading(calculatePointToCenterOfReefHeading());
            swerve.field.getObject("Reef").setPose(Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef"));
            absDrive.setHeading(calculatePointToTargetHeading(Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef"))); 
        }

        calculateL4ScoreAngle(Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef"), swerve.getPose());
        // absDrive.setHeading(calculatePointToTargetHeading(Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef")));
        // swerve.setFieldRelChassisSpeedsAndSkewCorrect(new ChassisSpeeds(1,0,0));
        
        if(StowButton){
            currentState = State.IDLE;
        }

        if(L1EjectButton){
            L1arm.runIntake(L1IntakeConstants.SCORE_SPEED);
        }
        else{
            L1arm.runIntake(L1IntakeSpeed);
        }

        //
        // State Machine 
        switch (currentState) {
            case IDLE:
                
                // L1 ~90, 
                L1arm.setArmAngle(L1ArmConstants.STOWED);
                L4arm.setArmAngle(L4ArmConstants.STOWED);
                
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.NOPICKUP_CURRENT_THRESHOULD;
                
                L1IntakeSpeed = L1IntakeConstants.HOLDING_SPEED;
                // change state?
                if(inAuto){
                    if(autoL4HumanLoadTrigger){
                        currentState = State.L4_INTAKING;
                    }
                    if(autoL4ScoreTrigger){
                        currentState = State.L4_SCORING;
                    }
                }
                else{

                    if(L1arm.atGoal() && L4arm.atGoal()){
                        if(L1IntakeButton){
                            currentState = State.L1_INTAKING;
                        }
        
                        if(L4IntakeButton){
                            currentState = State.L4_INTAKING;
                            
                        }
                        if(L1HumanLoadButton){
                            currentState = State.L1_HUMAN_LOADING;
                        }
        
                        if(L1ScoreButton){
                            currentState = State.L1_SCORE_POSITIONING;
                        }
        
                        if(L4ScoreButton){
                            currentState = State.L4_SCORING;
                        }
                    }
                }
                

                if(coralInL1){
                    cyclesIntaking += 1;
                    if(cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.L1_HOLDING;
                    }
                }
                else{
                    cyclesIntaking = 0;
                }
                

            break;


            case L1_INTAKING:
                
                // Move L1 to intaking pos, make sure L4 is stowed
                L1arm.setArmAngle(L1ArmConstants.INTAKING);
                L4arm.setArmAngle(L4ArmConstants.STOWED);
                // wait for spike in current that means we have coral
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.INTAKING_CURRENT_THRESHOULD;
                L1IntakeSpeed = L1IntakeButton ? L1IntakeConstants.INTAKE_SPEED : 0;
                if (coralInL1){
                    if( cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.L1_HOLDING;
                    }
                    cyclesIntaking += 1;
                }
                else{
                    cyclesIntaking = 0;
                }

                if(L4IntakeButton){
                    currentState = State.IDLE;
                }
                if(L1HumanLoadButton){
                    currentState = State.L1_HUMAN_LOADING;
                }

            break;
            case L1_HUMAN_LOADING:

                L1arm.setArmAngle(L1ArmConstants.HUMANLOADING);
                L1IntakeSpeed = L1HumanLoadButton ? L1IntakeConstants.INTAKE_SPEED : 0;
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.INTAKING_CURRENT_THRESHOULD;

                if (coralInL1){
                    if( cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.L1_HOLDING;
                    }
                    cyclesIntaking += 1;
                }
                else{
                    cyclesIntaking = 0;
                }
                 
                if(!L1HumanLoadButton){
                    currentState = State.IDLE;
                }
            break;

            case L1_HOLDING:
                // Stow L1 
                L1arm.setArmAngle(L1ArmConstants.STOWED);
                L4arm.setArmAngle(L4ArmConstants.STOWED);

                L1IntakeSpeed = L1IntakeConstants.HOLDING_SPEED;
                
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.NOPICKUP_CURRENT_THRESHOULD;                
                

                if(L1ScoreButton){

                    currentState = State.L1_SCORE_POSITIONING;
                }
                
                if(HandOffButton){
                    currentState = State.POSITIONING_HANDOFF;
                }

                if(!coralInL1){
                    cyclesIntaking += 1;
                    if(cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.IDLE;
                    }
                }
                else{
                    cyclesIntaking = 0;
                }

            break;


            case L1_SCORE_POSITIONING:
                // Move to scoring position then stop

                L1arm.setArmAngle(L1ArmConstants.SCORING);

                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.NOPICKUP_CURRENT_THRESHOULD; 

                if(L1arm.atGoal() && L1ScoreButton){
                    currentState = State.L1_SCORING;
                }
                if(!coralInL1){
                    cyclesIntaking += 1;
                    if(cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.IDLE;
                    }
                }
                else{
                    cyclesIntaking = 0;
                }
                

            break;


            case L1_SCORING:
                // Move the roller forward
                L1arm.runIntake(L1IntakeConstants.SCORE_SPEED);

                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.RELEASED_CURRENT_THRESHOULD;
                if(!coralInL1){
                    releasedCoralCycles += 1;
                    if(releasedCoralCycles >= L1IntakeConstants.CYCLE_RELEASED_THRESHOLD){
                        currentState = State.IDLE;
                    }
                }
                else{
                    releasedCoralCycles = 0;
                }
                
                if(!L1ScoreButton){
                    currentState = State.L1_SCORE_POSITIONING;
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
                L4arm.setArmAngle(L4ArmConstants.INTAKING);
                
                if(inAuto){
                    if(!autoL4HumanLoadTrigger){
                        currentState = State.IDLE;
                    }
                }
                else{
                    if(!L4IntakeButton){
                        currentState = State.IDLE;
    
                    }
                }
               

            break;

            case L4_SCORING:
                L1arm.setArmAngle(L1ArmConstants.STOWED);
                L4arm.setArmAngle(L4ArmConstants.SCORING);
                //L4arm.setArmAngle(L4ScoreAngle);

                
                if(inAuto){
                    if(!autoL4ScoreTrigger){
                        currentState = State.IDLE;
                    }
                }
                else{
                    if(!L4ScoreButton){
                        currentState = State.IDLE;
                    }
                }
            
            break;

    }
    }

    private Rotation2d calculateL4ScoreAngle(Pose2d target, Pose2d robotPose){
        Translation2d shoulderFieldPose = new Translation2d(
            robotPose.getX() + Math.cos(robotPose.getRotation().getRadians() + Math.atan2(L4ArmConstants.SHOULDER_LOCATION.getY(), L4ArmConstants.SHOULDER_LOCATION.getX())) * Math.hypot(L4ArmConstants.SHOULDER_LOCATION.getX(), L4ArmConstants.SHOULDER_LOCATION.getY()),
            robotPose.getY() + Math.sin(robotPose.getRotation().getRadians() + Math.atan2(L4ArmConstants.SHOULDER_LOCATION.getY(), L4ArmConstants.SHOULDER_LOCATION.getX())) * Math.hypot(L4ArmConstants.SHOULDER_LOCATION.getX(), L4ArmConstants.SHOULDER_LOCATION.getY()));
        double armDistanceFromTarget = Math.hypot(target.getX() - shoulderFieldPose.getX(), target.getY() - shoulderFieldPose.getY());
        
        swerve.field.getObject("ShoulderPositon").setPose(new Pose2d(shoulderFieldPose,Rotation2d.fromRadians(swerve.getPose().getRotation().getRadians() + Math.PI)));
        Rotation2d L4ScoreAngle = Rotation2d.fromRadians(Math.PI - Math.acos(armDistanceFromTarget/L4ArmConstants.ARM_LENGTH));
        if(L4ScoreAngle.getSin() * L4ArmConstants.ARM_LENGTH + L4ArmConstants.SHOULDER_LOCATION.getZ() < L4ArmConstants.MAX_SCORING_Z){
            return L4ScoreAngle;
        }
        else{
            return L4ArmConstants.MAX_SCORING_Z_ANGLE;
        }
        
    }
    private Rotation2d calculatePointToCenterOfReefHeading(){
        return Rotation2d.fromRadians(Math.atan2(
                    swerve.getPose().getY() - Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef").getY(), 
                    swerve.getPose().getX() - Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef").getX())
                    );
              
    }
    private Rotation2d calculatePointToTargetHeading(Pose2d target){
        
        double distanceFromTarget = Math.hypot(swerve.getPose().getX() - target.getX(), swerve.getPose().getY() - target.getY());
        Rotation2d heading = new Rotation2d();
        if (distanceFromTarget > L4ArmConstants.SHOULDER_LOCATION.getY()){
            if(swerve.getAlliance() == Alliance.Blue){
                heading = Rotation2d.fromRadians(
                Math.atan2(target.getY() - swerve.getPose().getY(), target.getX() - swerve.getPose().getX()) - Math.asin((-L4ArmConstants.SHOULDER_LOCATION.getY())/(Math.hypot(target.getX() - swerve.getPose().getX(), target.getY() - swerve.getPose().getY()))) + Math.PI
                );
            }
            else{
                heading = Rotation2d.fromRadians(
                Math.atan2(target.getY() - swerve.getPose().getY(), target.getX() - swerve.getPose().getX()) - Math.asin((L4ArmConstants.SHOULDER_LOCATION.getY())/(Math.hypot(target.getX() - swerve.getPose().getX(), target.getY() - swerve.getPose().getY())))
                );
            }


        }
        
        
        return heading;
    }
   

}