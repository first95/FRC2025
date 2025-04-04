// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.Constants.L4ArmConstants;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.L4Arm;
import frc.robot.subsystems.SwerveBase;

import java.sql.Driver;
import java.time.temporal.TemporalQuery;
import java.util.Arrays;
import java.util.Map;

import com.pathplanner.lib.auto.CommandUtil;

import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private final AutoFactory autoFactory;
  private final SwerveBase swerve;
  private final Trigger L4armAtGoal, L4armNotMoving;
  private final L4Arm L4arm;
  
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

 

  public Autos(SwerveBase swerve, L4Arm L4arm) {
    autoFactory = new AutoFactory(
      swerve::getPose,
      swerve::resetOdometry,
      swerve::followTrajectory,
      true,
      swerve);
    
    this.swerve = swerve;
    this.L4arm = L4arm;

    autoFactory.bind(
      "L4Score", 
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true))
      )
    );

    autoFactory.bind(
      "L4Intake",
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false))
      )
    );

    autoFactory.bind(
      "Stow",
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false))
      )
    );
    L4armAtGoal = new Trigger(() -> L4arm.atGoal());
    L4armNotMoving = new Trigger(() -> !L4arm.isMoving());
    
    SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false);
    SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false);
  }

  public AutoRoutine Diamond(){
    AutoRoutine routine = autoFactory.newRoutine("Diamond");
    

    AutoTrajectory Diamond = routine.trajectory("Diamond");

    //print trajectory
    swerve.field.getObject("autoTrajectory").setPoses(Diamond.getRawTrajectory().getPoses());
    

    routine.active().onTrue(
        Commands.sequence(
            Diamond.resetOdometry(),
            Diamond.cmd()
        )
    );
    return routine;
  }
  public AutoRoutine Diamond2(){
    AutoRoutine routine = autoFactory.newRoutine("Diamond 2");
    

    AutoTrajectory Diamond2 = routine.trajectory("Diamond 2");

    //print trajectory
    swerve.field.getObject("autoTrajectory").setPoses(Diamond2.getRawTrajectory().getPoses());
    

    routine.active().onTrue(
        Commands.sequence(
            Diamond2.resetOdometry(),
            Diamond2.cmd()
        )
    );
    return routine;
  }
  
  public AutoRoutine L4HumanLoadAndScore(){
    AutoRoutine routine = autoFactory.newRoutine("L4HumanLoadAndScore");

    AutoTrajectory humanLoadAndScore = routine.trajectory("L4HumanLoadAndScore");
    AutoTrajectory ScoreToHumanLoad = routine.trajectory("L4ScoreandHumanLoad");

    swerve.field.getObject("autoTrajectory").setPoses(humanLoadAndScore.getRawTrajectory().getPoses());
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
        humanLoadAndScore.resetOdometry(),
        humanLoadAndScore.cmd()
      )
    );
    

    Trigger atHumanLoad = ScoreToHumanLoad.done();
    atHumanLoad.onTrue(new WaitCommand(10).andThen(humanLoadAndScore.cmd()));

    Trigger atScoring = humanLoadAndScore.done();
    atScoring.onTrue(new WaitCommand(1).andThen(ScoreToHumanLoad.cmd()));

    routine.active().onFalse(new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)));
    return routine;
  }
  
  
  public AutoRoutine ModularAuto(){
    AutoRoutine routine = autoFactory.newRoutine("ModularAuto");
    String[] posTargets = getPosTargets();
    
    Pose2d[] fullTrajectory = {};    
    if (posTargets != null && posTargets.length >= 2){
      AutoTrajectory[] trajectories = new AutoTrajectory[posTargets.length - 1];
      


      //load trajectorys based on posTargets
      for(int n = 0; n < trajectories.length; n++){
        trajectories[n] = routine.trajectory(posTargets[n] + " - " + posTargets[n+1]);
        
        Pose2d[] trajectoryPose2dList = trajectories[n].getRawTrajectory().getPoses();
        fullTrajectory = Arrays.copyOf(fullTrajectory, fullTrajectory.length + trajectoryPose2dList.length );
        System.arraycopy(trajectoryPose2dList, 0, fullTrajectory, fullTrajectory.length - trajectoryPose2dList.length , trajectoryPose2dList.length);
      } 
      
      
      //When the routine starts run the first trajectory
      routine.active().onTrue(
        Commands.sequence(
          //trajectories[0].resetOdometry(),
          new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
          new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false)),
          new AlignToPose(trajectories[0].getInitialPose().get().plus(new Transform2d(new Translation2d(0.4,0),new Rotation2d())), swerve),
          new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true)),
          new AlignToPose(trajectories[0].getInitialPose().get(), swerve),
          new WaitUntilCommand(() -> !L4arm.isMoving()),
          new WaitCommand(Auton.SCORING_WAIT_TIME),
          new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false)),
          trajectories[0].cmd()
        )
      );
      //go through all trajectorys and run them one after another
      for(int n = 0; n < trajectories.length - 1; n++){
        //if the position target is at the reef wait the scoring time
        if(posTargets[n+1].charAt(0) == 'R'){
          trajectories[n].done().onTrue(
            new WaitUntilCommand(() -> !L4arm.isMoving())
            .andThen(new WaitCommand(Auton.SCORING_WAIT_TIME))
            .andThen(new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false)))
            .andThen(trajectories[n+1].cmd()));
        }
        //if the position target is at a loading station wait the humanload time
        else if(posTargets[n+1].charAt(0) == 'L'){
          trajectories[n].done().onTrue(
            Commands.deadline(new WaitCommand(Auton.HUMANLOAD_WAIT_TIME), new AlignToPose(posTargets[n+1], swerve))
            .andThen(trajectories[n+1].cmd()))
            ;
        }
      }
      
      swerve.field.getObject("autoTrajectory").setPoses(fullTrajectory);
      swerve.field.getObject("target").setPose(trajectories[0].getInitialPose().get());
    }
    
    return routine;  
  }

  public AutoRoutine threeCoralAutoLeftSide(){
    SmartDashboard.putString("currentModularAuto", "R50,L0,R40,L0,R41,L0,R50,L0,");
    
    return ModularAuto();  
  }
  public AutoRoutine threeCoralAutoRightSide(){
    SmartDashboard.putString("currentModularAuto", "R11,L1,R21,L1,R20,L1,R10,L1,");
    
    return ModularAuto();  
  }
  private String[] getPosTargets(){
    String currentModularAuto = SmartDashboard.getString("currentModularAuto", "");
    String[] posTargets = {};
    String posTarget = "";
    if(currentModularAuto.length() > 3){
      for(int character = 0; character < currentModularAuto.length(); character ++){
      
        if(currentModularAuto.charAt(character) == ','){
          int posTargetsLength = posTargets.length;
          posTargets = Arrays.copyOf(posTargets,posTargetsLength + 1);
          posTargets[posTargetsLength] = posTarget;
          posTarget = "";
        }
        else{
          posTarget += currentModularAuto.charAt(character);
        }
      }
      SmartDashboard.putStringArray("posTargets", posTargets);
      return posTargets;
      
    }
    else{
      return new String[0];
    }
    
    
    
  }
 
}
