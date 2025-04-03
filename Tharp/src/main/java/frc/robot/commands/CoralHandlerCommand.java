package frc.robot.commands;


import static edu.wpi.first.units.Units.Rotation;

import java.security.spec.ECPublicKeySpec;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.L1Arm;
import frc.robot.subsystems.L4Arm;
import frc.robot.subsystems.SwerveBase;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import frc.robot.Constants.L1ArmConstants;
import frc.robot.Constants.L1IntakeConstants;
import frc.robot.Constants.L4ArmConstants;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.Constants;


public class CoralHandlerCommand extends Command {
    
    private final BooleanSupplier 
        L1IntakeButtonSupplier, 
        L1EjectButtonSupplier, 
        L1InjectButtonSupplier,
        L4IntakeButtonSupplier,
        L4ScoreButtonSupplier, 
        HandOffButtonSupplier, 
        L1ScoreButtonSupplier,
        StowButtonSupplier, 
        L1HumanLoadingSupplier,
        pointToReefButtonSupplier,
        alignWithHumanLoadButtonSupplier,
        climbButtonSupplier,
        IgnoreL1PositionButtonSupplier;

    private final L1Arm L1arm;
    private final L4Arm L4arm;
    private final Climber climber;
    private final SwerveBase swerve;
    private final AbsoluteDrive absDrive;
    private enum State{
        IDLE, L1_INTAKING, L1_SCORE_POSITIONING, 
        L4_POSITIONING_HANDOFF,L1_POSITIONING_HANDOFF, PERFORMING_HANDOFF, 
        L4_INTAKING, L4_SCORING, L1_HUMAN_LOADING, CLIMBING_L4_POSITIONING,CLIMBING_L1_POSITIONING;
    }

    private boolean 
        L1IntakeButton, 
        L1EjectButton, 
        L1InjectButton,
        L4IntakeButton, 
        HandOffButton,
        L1ScoreButton, 
        L4ScoreButton, 
        StowButton, 
        L1HumanLoadButton,
        pointToReefButton,
        alignWithHumanLoadButton,
        climbButton,
        IgnoreL1PositionButton;
    
    private boolean 
        autoL4HumanLoadTrigger,
        autoL4ScoreTrigger,
        autoL1ScoreTrigger,
        autoHandOffTrigger,
        inAuto;


    private boolean coralInL1 = false;
    private boolean IgnoreL1Position = false;
    private boolean releasedCoral = false; 
    private int releasedCoralCycles = 0;
    private double IntakeCurrent;
    private Rotation2d L4ScoreAngle,L1ScoreAngle;
    private int cyclesIntaking, cyclesPositioningHandOff;
    private double L1IntakeSpeed;
    private State currentState = State.IDLE;
    private Pose2d L4Target, L4ScorePose, LeftScorePose,RightScorePose;

        public CoralHandlerCommand(
            BooleanSupplier L1IntakeButtonSupplier, 
            BooleanSupplier L1EjectButtonSupplier, 
            BooleanSupplier L1InjectButtonSupplier,
            BooleanSupplier L4IntakeButtonSupplier,
            BooleanSupplier L4ScoreButtonSupplier, 
            BooleanSupplier HandOffButtonSupplier, 
            BooleanSupplier L1ScoreButtonSupplier, 
            BooleanSupplier StowButtonSupplier, 
            BooleanSupplier L1HumanLoadingSupplier,
            BooleanSupplier pointToReefButtonSupplier,
            BooleanSupplier alignWithHumanLoadButtonSupplier, 
            BooleanSupplier climbButtonSupplier,
            BooleanSupplier IgnoreL1PositionButtonSupplier,
            L1Arm L1arm,
            L4Arm L4arm,
            Climber climber,
            SwerveBase swerve,
            AbsoluteDrive absDrive){
            
    
    
    
            this.L1IntakeButtonSupplier = L1IntakeButtonSupplier;
            this.L1EjectButtonSupplier = L1EjectButtonSupplier;
            this.L1InjectButtonSupplier = L1InjectButtonSupplier;
            this.L1HumanLoadingSupplier = L1HumanLoadingSupplier;
            this.StowButtonSupplier = StowButtonSupplier;
            this.L4IntakeButtonSupplier = L4IntakeButtonSupplier;

            this.HandOffButtonSupplier = HandOffButtonSupplier;
            this.L1ScoreButtonSupplier =L1ScoreButtonSupplier;

            this.L4ScoreButtonSupplier = L4ScoreButtonSupplier;
            this.pointToReefButtonSupplier = pointToReefButtonSupplier;
            this.alignWithHumanLoadButtonSupplier = alignWithHumanLoadButtonSupplier;
            this.climbButtonSupplier = climbButtonSupplier;
            this.IgnoreL1PositionButtonSupplier = IgnoreL1PositionButtonSupplier;

        
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
        L1InjectButton = L1InjectButtonSupplier.getAsBoolean();
        L1HumanLoadButton = L1HumanLoadingSupplier.getAsBoolean();
        StowButton = StowButtonSupplier.getAsBoolean();

        L4IntakeButton = L4IntakeButtonSupplier.getAsBoolean();
        L4ScoreButton = L4ScoreButtonSupplier.getAsBoolean();
        
        HandOffButton = HandOffButtonSupplier.getAsBoolean();
        climbButton = climbButtonSupplier.getAsBoolean();

        L1ScoreButton = L1ScoreButtonSupplier.getAsBoolean();

        IgnoreL1PositionButton = IgnoreL1PositionButtonSupplier.getAsBoolean();
        
        alignWithHumanLoadButton = alignWithHumanLoadButtonSupplier.getAsBoolean();
        

        inAuto = SmartDashboard.getBoolean(Constants.Auton.AUTO_ENABLED_KEY, false);
        autoL1ScoreTrigger = SmartDashboard.getBoolean(Constants.Auton.L1SCORE_KEY, false);
        autoL4HumanLoadTrigger = SmartDashboard.getBoolean(Constants.Auton.L4HUMANLOAD_KEY, false);
        autoL4ScoreTrigger = SmartDashboard.getBoolean(Constants.Auton.L4SCORE_KEY, false);
        autoHandOffTrigger = SmartDashboard.getBoolean(Constants.Auton.AUTO_HANDOFF_KEY, false);

        pointToReefButton = pointToReefButtonSupplier.getAsBoolean();

        LeftScorePose = findLeftScorePose();
        RightScorePose = findRightScorePose();      
        L1ScoreAngle = Math.abs(swerve.getPose().getRotation().minus(calculatePointToCenterOfReefHeading()).getRadians()) < Math.PI/2 ? 
            L1ArmConstants.SCORING_OF_BACK:
            L1ArmConstants.SCORING_OF_FRONT;
        
        SmartDashboard.putNumber("L1ScoreAngle", L1ScoreAngle.getDegrees());
        swerve.field.getObject("L1ScorePose").setPose(RightScorePose);
        if(pointToReefButton){
            //absDrive.setHeading(calculatePointToCenterOfReefHeading());
            absDrive.setHeading(calculatePointToTargetHeading(L4Target)); 
            L4ScoreAngle =  L4ArmConstants.SCORING;//calculateL4ScoreAngle(L4Target);
            
        }
        else{
            L4Target = findClosestL4Target();
            //L4Target = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("Reef");
            L4ScorePose = findScoringPose(L4Target);
            L4ScoreAngle = L4ArmConstants.SCORING;
            swerve.field.getObject("Target").setPose(L4Target);
            swerve.field.getObject("Scoring").setPose(L4ScorePose);
        }

    

        if(alignWithHumanLoadButton){
            absDrive.setHeading(Rotation2d.fromDegrees((swerve.getAlliance() == Alliance.Blue ? 1:-1)*(swerve.getPose().getY() < Constants.FIELD_WIDTH/2 ? 1 : -1) * Constants.Auton.LINEUP_TO_HUMANLOADANGLE + 180) );
        }

        
        
        if(StowButton){
            currentState = State.IDLE;
        }

        if(L1EjectButton){
            L1arm.runIntake(-1);
        }
        else if(L1InjectButton){
            L1arm.runIntake(1);
        }
        else{
            L1arm.runIntake(L1IntakeSpeed);
        }
        if(IgnoreL1PositionButton){
            IgnoreL1Position = true;
        }
        if(IgnoreL1Position){
            L1arm.setControleffort(0);
            if(L4IntakeButton){
                currentState = State.L4_INTAKING;
            } 
            if(L4ScoreButton){
                currentState = State.L4_SCORING;
            }
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
                    if(autoHandOffTrigger){
                        currentState = State.L4_POSITIONING_HANDOFF;
                    }
                }
                else{

                    if(L1arm.atGoal() && L4arm.atGoal()){
                        if(L1IntakeButton){
                            currentState = State.L1_INTAKING;
                        }
                        if(climbButton){
                            currentState = State.CLIMBING_L1_POSITIONING;
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
                        if(HandOffButton){
                            currentState = State.L4_POSITIONING_HANDOFF;
                        }
                    }
                    
                }
                

                // if(coralInL1){
                //     cyclesIntaking += 1;
                //     if(cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                //         currentState = State.L1_SCORE_POSITIONING;
                //     }
                // }
                // else{
                //     cyclesIntaking = 0;
                // }
                

            break;


            case L1_INTAKING:
                
                // Move L1 to intaking pos, make sure L4 is stowed
                L1arm.setArmAngle(L1ArmConstants.INTAKING);
                L4arm.setArmAngle(L4ArmConstants.STOWED);
                // wait for spike in current that means we have coral
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.NOPICKUP_CURRENT_THRESHOULD;
                L1IntakeSpeed = L1IntakeButton ? L1IntakeConstants.INTAKE_SPEED : L1IntakeConstants.HOLDING_SPEED;
                if (coralInL1){
                    if( cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD && !L1IntakeButton){
                        currentState = State.IDLE;
                    }
                    cyclesIntaking += 1;
                }
                else{
                    cyclesIntaking = 0;
                }
                
                if(L1HumanLoadButton){
                    currentState = State.L1_HUMAN_LOADING;
                }

                if(L4ScoreButton){
                    currentState = State.IDLE;
                }
                if(L4IntakeButton){ 
                    currentState = State.IDLE;
                } 
                if(L1ScoreButton){
                    currentState = State.L1_SCORE_POSITIONING;
                }
                if(climbButton){
                    currentState = State.CLIMBING_L1_POSITIONING;
                }
                if(HandOffButton){
                    currentState = State.IDLE;
                }                
                
            break;
            case L1_HUMAN_LOADING:

                L1arm.setArmAngle(L1ArmConstants.HUMANLOADING);
                L1IntakeSpeed = L1HumanLoadButton ? L1IntakeConstants.INTAKE_SPEED : 0;
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.INTAKING_CURRENT_THRESHOULD;

                 
                if(!L1HumanLoadButton){
                    currentState = State.IDLE;
                } 
                if(inAuto){
                    if(autoL4HumanLoadTrigger){
                        currentState = State.L4_INTAKING;
                    }
                    if(autoL4ScoreTrigger){
                        currentState = State.L4_SCORING;
                    }
                }

                if(L4ScoreButton){
                    currentState = State.IDLE;
                }

                if(L4IntakeButton){
                    currentState = State.L4_INTAKING;
                    
                }
            break;



            case L1_SCORE_POSITIONING:
                // Move to scoring position then stop

                L1arm.setArmAngle(L1ScoreAngle);

                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.NOPICKUP_CURRENT_THRESHOULD; 
                //L1IntakeSpeed = L1ScoreButton ? L1IntakeConstants.SCORE_SPEED : 0;

                
                if(inAuto){
                    if(autoL1ScoreTrigger){
                        L1IntakeSpeed = L1IntakeConstants.SCORE_SPEED;
                    }
                    else{
                        L1IntakeSpeed = L1IntakeConstants.HOLDING_SPEED;
                    }
                }
                else{
                    if(L1arm.atGoal()){
                       
                        if(L1ScoreButton){
                            L1IntakeSpeed = L1IntakeConstants.SCORE_SPEED;
                        }
                        else{
                            L1IntakeSpeed = L1IntakeConstants.HOLDING_SPEED;
                        }
                    }
                    if(L4ScoreButton){
                        currentState = State.IDLE;
                    }
                    if(L4IntakeButton){ 
                        currentState = State.IDLE;
                    } 
                    if(L1IntakeButton){
                        currentState = State.L1_INTAKING;
                    }
                    if(climbButton){
                        currentState = State.CLIMBING_L1_POSITIONING;
                    }
                    if(L1HumanLoadButton){
                        currentState = State.IDLE;
                    }
                    if(HandOffButton){
                        currentState = State.IDLE;
                    }
                }

            break;


            case L4_POSITIONING_HANDOFF:
                L4arm.setArmAngle(L4ArmConstants.HAND_OFF);
                
                absDrive.setBrake(true);
                if(inAuto){
                    if(!autoHandOffTrigger){
                        currentState = State.IDLE;
                        absDrive.setBrake(false);
                    }
                }
                else{
                    if(!HandOffButton){
                        currentState = State.IDLE;
                        absDrive.setBrake(false);
                    }
                }
                cyclesPositioningHandOff = 0; 
                if(L4arm.atGoal()){
                    currentState = State.L1_POSITIONING_HANDOFF;
                }
            break;

            case L1_POSITIONING_HANDOFF:
                L1arm.setArmAngle(L1ArmConstants.HAND_OFF);
                L4arm.setArmAngle(L4ArmConstants.HAND_OFF);

                absDrive.setBrake(true);

                cyclesPositioningHandOff += 1;
                if(inAuto){
                    if(!autoHandOffTrigger){
                        currentState = State.IDLE;
                        absDrive.setBrake(false);
                    }
                }
                else{
                    if(!HandOffButton){
                        currentState = State.IDLE;
                        absDrive.setBrake(false);
                    }
                }
                if(L1arm.atGoal() && L4arm.atGoal() && cyclesPositioningHandOff >= L1ArmConstants.HAND_OFF_SETTLING_TIME){
                    currentState = State.PERFORMING_HANDOFF;
                } 

            break;


            case PERFORMING_HANDOFF:
                // Less resistance then go to L4 holding
                L1IntakeSpeed = L1IntakeConstants.HAND_OFF_SPEED;

                absDrive.setBrake(true);
                if(inAuto){
                    if(autoL4ScoreTrigger){
                        currentState = State.L4_SCORING;
                        absDrive.setBrake(false);
                    }
                    if(!autoHandOffTrigger){
                        currentState = State.IDLE;
                        absDrive.setBrake(false);
                    }
                }
                else{
                    if(!HandOffButton){
                        currentState = State.IDLE;
                        absDrive.setBrake(false);
                    }
                }
            break;


            case L4_INTAKING:
                // L4 intaking pos
                // L1 backstop ~95 deg
                L4arm.setArmAngle(L4ArmConstants.STOWED);
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
                L4arm.setArmAngle(L4ArmConstants.SCORING);
                L4arm.setArmAngle(L4ScoreAngle);

                
                if(inAuto){
                    //L4ScoreAngle = calculateL4ScoreAngle(L4Target);
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
            case CLIMBING_L4_POSITIONING:
                L1arm.setArmAngle(L1ArmConstants.CLIMBING);
                L4arm.setArmAngle(L4ArmConstants.CLIMBING);
                L1IntakeSpeed = 0;

                if(L4IntakeButton || L1IntakeButton || L1HumanLoadButton){
                    currentState = State.IDLE;
                }
            break;
            case CLIMBING_L1_POSITIONING:
                L1arm.setArmAngle(L1ArmConstants.CLIMBING);
                L1IntakeSpeed = 0;
                
                if(L1arm.atGoal()){
                    currentState = State.CLIMBING_L4_POSITIONING;
                }
                if(L4IntakeButton || L1IntakeButton || L1HumanLoadButton){
                    currentState = State.L1_INTAKING;
                }
            break;
    }
    }

    private Rotation2d calculateL4ScoreAngle(Pose2d target){
        
        Pose2d shoulderFieldPose = swerve.getPose().plus(L4ArmConstants.SHOULDER_TRANSFORM);
        double armDistanceFromTarget = Math.hypot(target.getX() - shoulderFieldPose.getX(), target.getY() - shoulderFieldPose.getY());
        
        swerve.field.getObject("ShoulderPositon").setPose(shoulderFieldPose);
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
        Pose2d shoulderFieldPose = swerve.getPose().plus(L4ArmConstants.SHOULDER_TRANSFORM);
        swerve.field.getObject("ShoulderPositon").setPose(shoulderFieldPose);

        double distanceFromTarget = Math.hypot(swerve.getPose().getX() - target.getX(), swerve.getPose().getY() - target.getY());
        Rotation2d heading = Rotation2d.fromDegrees(0);
        if (distanceFromTarget > L4ArmConstants.SHOULDER_LOCATION.getY()){
            if(swerve.getAlliance() == Alliance.Blue){
                heading = Rotation2d.fromRadians(// creates a circle of radius Shoulder Relative Y and then finds the heaing to aim tangent to that circle
                Math.atan2(target.getY() - swerve.getPose().getY(), target.getX() - swerve.getPose().getX()) - Math.asin((-L4ArmConstants.SHOULDER_LOCATION.getY())/(Math.hypot(target.getX() - swerve.getPose().getX(), target.getY() - swerve.getPose().getY()))) + Math.PI
                );
            }
            else{// flip when on red alliance
                heading = Rotation2d.fromRadians(
                Math.atan2(target.getY() - swerve.getPose().getY(), target.getX() - swerve.getPose().getX()) + Math.asin((L4ArmConstants.SHOULDER_LOCATION.getY())/(Math.hypot(target.getX() - swerve.getPose().getX(), target.getY() - swerve.getPose().getY())))
                );
            }


        }
        
       return heading;
       
    }
    private Pose2d findClosestL4Target(){
        Pose2d shoulderFieldPose = swerve.getPose().plus(L4ArmConstants.SHOULDER_TRANSFORM);

        Pose2d closestL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 0);
        Pose2d currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 0);
        for(int side = 0; side <= 5; side ++){
            for(int L4Pole = 0; L4Pole <= 1;L4Pole ++){
                currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + side + L4Pole);
                if(Math.hypot(currentL4Pole.getX() - shoulderFieldPose.getX(), currentL4Pole.getY() - shoulderFieldPose.getY()) < Math.hypot(closestL4Pole.getX() - shoulderFieldPose.getX(), closestL4Pole.getY() - shoulderFieldPose.getY())){
                    closestL4Pole = currentL4Pole;
                }
            }
        }
        return closestL4Pole;
        
    }
    private Pose2d findScoringPose(Pose2d L4Target){
        return new Pose2d(new Translation2d(L4Target.getX(),L4Target.getY()),L4Target.getRotation().rotateBy(Rotation2d.fromDegrees(180)))
            .plus(
                (L4ArmConstants.SHOULDER_TRANSFORM.plus(
                    new Transform2d(-L4ArmConstants.SCORING.getCos() * L4ArmConstants.ARM_LENGTH,
                    0.0,
                    Rotation2d.fromDegrees(0)))));
        
     }
     private Pose2d findRightScorePose(){
        Pose2d shoulderFieldPose = swerve.getPose().plus(L4ArmConstants.SHOULDER_TRANSFORM);

        Pose2d closestL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 0);
        Pose2d currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 0);
        for(int side = 0; side <= 5; side ++){
            currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + side + 0);
            if(Math.hypot(currentL4Pole.getX() - shoulderFieldPose.getX(), currentL4Pole.getY() - shoulderFieldPose.getY()) < Math.hypot(closestL4Pole.getX() - shoulderFieldPose.getX(), closestL4Pole.getY() - shoulderFieldPose.getY())){
                closestL4Pole = currentL4Pole;
            }
        }
        // if(Math.abs(swerve.getPose().getRotation().getRadians() - calculatePointToCenterOfReefHeading().getRadians()) < Math.PI/2){
        //     return findScoringPose(closestL4Pole);
        // }
        // else{
        //     return new Pose2d(findScoringPose(closestL4Pole).getTranslation(),findScoringPose(closestL4Pole).getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        // }
        return findScoringPose(closestL4Pole); 
    }
    private Pose2d findLeftScorePose(){
        Pose2d shoulderFieldPose = swerve.getPose().plus(L4ArmConstants.SHOULDER_TRANSFORM);

        Pose2d closestL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 1);
        Pose2d currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 1);
        for(int side = 0; side <= 5; side ++){
            currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + side + 1);
            if(Math.hypot(currentL4Pole.getX() - shoulderFieldPose.getX(), currentL4Pole.getY() - shoulderFieldPose.getY()) < Math.hypot(closestL4Pole.getX() - shoulderFieldPose.getX(), closestL4Pole.getY() - shoulderFieldPose.getY())){
                closestL4Pole = currentL4Pole;
            }
        }
        // if(Math.abs(swerve.getPose().getRotation().getRadians() - calculatePointToCenterOfReefHeading().getRadians()) < Math.PI/2){
        //     return findScoringPose(closestL4Pole);
        // }
        // else{
        //     return new Pose2d(findScoringPose(closestL4Pole).getTranslation(),findScoringPose(closestL4Pole).getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        // }
        return findScoringPose(closestL4Pole);
    }

    public Trigger getCoralInL1(){
        return new Trigger(() -> coralInL1);
    }
    public Trigger getCoralNotInL1(){
        return new Trigger(() -> !coralInL1);
    }
    public Command LeftL4AutoScore(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> LeftScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true)),
            new WaitCommand(Constants.Auton.SCORING_WAIT_TIME + L4ArmConstants.TIME_TO_SCORING),
            cancelL4AutoScore()
            );
    }
    public Command RightL4AutoScore(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> RightScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true)),
            new WaitCommand(Constants.Auton.SCORING_WAIT_TIME + L4ArmConstants.TIME_TO_SCORING),
            cancelL4AutoScore()
            );
    }

    public Command LefthandOffAndL4(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> LeftScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_HANDOFF_KEY, true)),
            new WaitCommand(Constants.Auton.HANDOFF_WAIT_TIME),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_HANDOFF_KEY, false)),
            LeftL4AutoScore()
        );
    }
    public Command RighthandOffAndL4(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> RightScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_HANDOFF_KEY, true)),
            new WaitCommand(Constants.Auton.HANDOFF_WAIT_TIME),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_HANDOFF_KEY, false)),
            RightL4AutoScore()
        );
    }
    public Command cancelL4AutoScore(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_HANDOFF_KEY, false))
        );
    }
    public Command cancelL1AutoScore(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L1SCORE_KEY, false))
        );
    }
    public Command L1AutoScore(){
        return 
        Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> RightScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L1SCORE_KEY, true)),
            new WaitCommand(Constants.Auton.SCORING_WAIT_TIME + L4ArmConstants.TIME_TO_SCORING),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L1SCORE_KEY, false))
            );
    }

}