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
        L4IntakeButtonSupplier,
        L4ScoreButtonSupplier, 
        HandOffButtonSupplier, 
        L1ScoreButtonSupplier,
        StowButtonSupplier, 
        L1HumanLoadingSupplier,
        pointToReefButtonSupplier,
        alignWithHumanLoadButtonSupplier,
        climbButtonSupplier;

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
        L4IntakeButton, 
        HandOffButton,
        L1ScoreButton, 
        L4ScoreButton, 
        StowButton, 
        L1HumanLoadButton,
        pointToReefButton,
        alignWithHumanLoadButton,
        climbButton;
    
    private boolean 
        autoL4HumanLoadTrigger,
        autoL4ScoreTrigger,
        autoL1ScoreTrigger,
        inAuto;


    private boolean coralInL1 = false;
    private boolean releasedCoral = false; 
    private int releasedCoralCycles = 0;
    private double IntakeCurrent;
    private Rotation2d L4ScoreAngle;
    private int cyclesIntaking;
    private double L1IntakeSpeed;
    private State currentState = State.IDLE;
    private Pose2d L4Target, L4ScorePose, L1ScorePose;
    private Trigger autoAlignTrigger;

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
            BooleanSupplier alignWithHumanLoadButtonSupplier, 
            BooleanSupplier climbButtonSupplier,
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
            this.alignWithHumanLoadButtonSupplier = alignWithHumanLoadButtonSupplier;
            this.climbButtonSupplier = climbButtonSupplier;

        
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
        climbButton = climbButtonSupplier.getAsBoolean();

        L1ScoreButton = L1ScoreButtonSupplier.getAsBoolean();

        alignWithHumanLoadButton = alignWithHumanLoadButtonSupplier.getAsBoolean();
        

        inAuto = SmartDashboard.getBoolean(Constants.Auton.AUTO_ENABLED_KEY, false);
        autoL1ScoreTrigger = SmartDashboard.getBoolean(Constants.Auton.L1SCORE_KEY, false);
        autoL4HumanLoadTrigger = SmartDashboard.getBoolean(Constants.Auton.L4HUMANLOAD_KEY, false);
        autoL4ScoreTrigger = SmartDashboard.getBoolean(Constants.Auton.L4SCORE_KEY, false);

        pointToReefButton = pointToReefButtonSupplier.getAsBoolean();

        L1ScorePose = findL1ScorePose();       
        
        swerve.field.getObject("L1ScorePose").setPose(L1ScorePose);
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
                    }
                }
                

                if(coralInL1){
                    cyclesIntaking += 1;
                    if(cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.L1_SCORE_POSITIONING;
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
                        currentState = State.L1_SCORE_POSITIONING;
                    }
                    cyclesIntaking += 1;
                }
                else{
                    cyclesIntaking = 0;
                }

                if(L1HumanLoadButton){
                    currentState = State.L1_HUMAN_LOADING;
                }

                if(L1arm.atGoal()){
                    if(L4ScoreButton){
                        currentState = State.L4_SCORING;
                    }
                    if(L4IntakeButton){
                        currentState = State.L4_INTAKING;
                    }
                }
                if(climbButton){
                    currentState = State.CLIMBING_L1_POSITIONING;
                }
                
            break;
            case L1_HUMAN_LOADING:

                L1arm.setArmAngle(L1ArmConstants.HUMANLOADING);
                L1IntakeSpeed = L1HumanLoadButton ? L1IntakeConstants.INTAKE_SPEED : 0;
                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.INTAKING_CURRENT_THRESHOULD;

                if (coralInL1){
                    if( cyclesIntaking >= L1IntakeConstants.CYCLE_INTAKING_THRESHOLD){
                        currentState = State.L1_SCORE_POSITIONING;
                    }
                    cyclesIntaking += 1;
                }
                else{
                    cyclesIntaking = 0;
                }
                 
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

                L1arm.setArmAngle(L1ArmConstants.SCORING);

                coralInL1 = L1arm.getIntakeCurrent() > L1IntakeConstants.NOPICKUP_CURRENT_THRESHOULD; 
                L1IntakeSpeed = L1ScoreButton ? L1IntakeConstants.SCORE_SPEED : 0;

                
                if(inAuto){
                    if(autoL1ScoreTrigger){
                        L1IntakeSpeed = L1IntakeConstants.SCORE_SPEED;
                    }
                    else{
                        L1IntakeSpeed = 0;
                    }
                }
                else{
                    if(L1arm.atGoal()){
                        if(L4ScoreButton){
                            currentState = State.L4_SCORING;
                        }
                        if(L4IntakeButton){
                            currentState = State.L4_INTAKING;
                        } 
                    }
                    if(L1IntakeButton){
                        currentState = State.L1_INTAKING;
                    }
                    if(climbButton){
                        currentState = State.CLIMBING;
                    }
                    if(L1ScoreButton){
                        L1IntakeSpeed = L1IntakeConstants.SCORE_SPEED;
                    }
                    else{
                        L1IntakeSpeed = 0;
                    }
                }

            break;


            case L4_POSITIONING_HANDOFF:
                L4arm.setArmAngle(L4ArmConstants.HAND_OFF);
                
                if(!HandOffButton){
                    currentState = State.IDLE;
                }
                if(L4arm.atGoal()){
                    currentState = State.L1_POSITIONING_HANDOFF;
                }
            break;

            case L1_POSITIONING_HANDOFF:
                L1arm.setArmAngle(L1ArmConstants.HAND_OFF);
                L4arm.setArmAngle(L4ArmConstants.HAND_OFF);
            
                if(!HandOffButton){
                    currentState = State.IDLE;
                }
                if(L1arm.atGoal() && L4arm.atGoal()){
                    currentState = State.PERFORMING_HANDOFF;
                } 

            break;


            case PERFORMING_HANDOFF:
                // Less resistance then go to L4 holding
                L1arm.runIntake(L1IntakeConstants.HAND_OFF_SPEED);
                if(!HandOffButton){
                    currentState = State.IDLE;
                }
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
                //L4arm.setArmAngle(L4ArmConstants.SCORING);
                L4arm.setArmAngle(L4ScoreAngle);

                
                if(inAuto){
                    L4ScoreAngle = calculateL4ScoreAngle(L4Target);
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
                L1arm.runIntake(0);

                if(L4IntakeButton || L1IntakeButton || L1HumanLoadButton){
                    currentState = State.IDLE;
                }
            break;
            case CLIMBING_L1_POSITIONING:
                L1arm.setArmAngle(L1ArmConstants.CLIMBING);
                L1arm.runIntake(0);
                if(L1arm.atGoal()){
                    currentState = State.CLIMBING_L4_POSITIONING;
                }
                if(L4IntakeButton || L1IntakeButton || L1HumanLoadButton){
                    currentState = State.IDLE;
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
        Rotation2d heading = new Rotation2d();
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
     private Pose2d findL1ScorePose(){
        Pose2d shoulderFieldPose = swerve.getPose().plus(L4ArmConstants.SHOULDER_TRANSFORM);

        Pose2d closestL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 0);
        Pose2d currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + 0 + 0);
        for(int side = 0; side <= 5; side ++){
            currentL4Pole = Constants.Auton.POSE_MAP.get(swerve.getAlliance()).get("R" + side + 0);
            if(Math.hypot(currentL4Pole.getX() - shoulderFieldPose.getX(), currentL4Pole.getY() - shoulderFieldPose.getY()) < Math.hypot(closestL4Pole.getX() - shoulderFieldPose.getX(), closestL4Pole.getY() - shoulderFieldPose.getY())){
                closestL4Pole = currentL4Pole;
            }
        }

        return findScoringPose(closestL4Pole);
    }
     
    public Command L4AutoScore(){
        return 
        Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> L4ScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true)),
            new WaitCommand(Constants.Auton.SCORING_WAIT_TIME + L4ArmConstants.TIME_TO_SCORING),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false)));
            
    }
    public Command cancelAutoScore(){
        return Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false))
        );
    }
    public Command L1AutoScore(){
        return 
        Commands.sequence(
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
            new AlignToPose(() -> L1ScorePose, swerve),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L1SCORE_KEY, true)),
            new WaitCommand(Constants.Auton.SCORING_WAIT_TIME + L4ArmConstants.TIME_TO_SCORING),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)),
            new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L1SCORE_KEY, false))
            );
    }

}