// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.commands.autocommands.FollowTrajectory;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveBase;

import java.sql.Driver;
import java.util.Arrays;
import java.util.Map;

import com.pathplanner.lib.auto.CommandUtil;

import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private final AutoFactory autoFactory;
  private final SwerveBase swerve;
  private final Command L1HumanLoad =
    new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, true)).andThen(
    new WaitCommand(Constants.Auton.HUMANLOAD_TIMEOUT)).andThen(
    new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false))
    );
  private final Command l1Score = Commands.sequence(
    new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true)),
    new WaitCommand(Constants.Auton.SCORE_TIMEOUT),
    new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, false)
  ));
  String runningAuto;
  
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

 

  public Autos(SwerveBase swerve) {
    autoFactory = new AutoFactory(
      swerve::getPose,
      swerve::resetOdometry,
      swerve::followTrajectory,
      true,
      swerve);
      this.swerve = swerve;
    autoFactory.bind(
      "L4ScoringPosition", 
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, false)),
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4SCORE_KEY, true))
      )
    );
    autoFactory.bind(
      "L4Intake",
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.L4HUMANLOAD_KEY, true)),
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
    AutoTrajectory ScoreToHumanLoad = routine.trajectory("scoreToHumanLoad");

    swerve.field.getObject("autoTrajectory").setPoses(humanLoadAndScore.getRawTrajectory().getPoses());
    routine.active().onTrue(
      Commands.sequence(
        new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, true)),
        new AlignToPose( humanLoadAndScore.getInitialPose().get() , swerve),
        humanLoadAndScore.cmd()
      )
    );
    

    Trigger atHumanLoad = ScoreToHumanLoad.done();
    atHumanLoad.onTrue(new WaitCommand(2));

    routine.active().onFalse(new InstantCommand(() -> SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, false)));
    return routine;
  }
  
  
  public AutoRoutine testModularAuto(){
    AutoRoutine routine = autoFactory.newRoutine("TestModularAuto");
    String[] posTargets = SmartDashboard.getStringArray("modularAutoTargets", null);
    
    
    Pose2d[] fullTrajectory = {};    
    if (posTargets != null && posTargets.length > 2){
      for (String Target : posTargets) {
        runningAuto += "- " + Target;
      }
      SmartDashboard.putString("runningAuto",runningAuto);
      AutoTrajectory[] trajectories = new AutoTrajectory[posTargets.length - 1];
      


      //load trajectorys based on posTargets
      for(int n = 0; n < trajectories.length; n++){
        trajectories[n] = routine.trajectory(posTargets[n] + " - " + posTargets[n+1]);
        Pose2d[] trajectoryPose2dList = trajectories[n].getRawTrajectory().getPoses();
        fullTrajectory = Arrays.copyOf(fullTrajectory, fullTrajectory.length + trajectoryPose2dList.length );
        System.arraycopy(trajectoryPose2dList, 0, fullTrajectory, fullTrajectory.length - trajectoryPose2dList.length , trajectoryPose2dList.length);
      } 
      //print Composite Trajectory
      
      
      //When the routine starts run the first trajectory
      routine.active().onTrue(
        Commands.sequence(
          trajectories[0].resetOdometry(),
          trajectories[0].cmd()
        )
      );
      
      //go through all trajectorys and run them one after another
      for(int n = 0; n < trajectories.length - 1; n++){
        trajectories[n].done().onTrue(trajectories[n+1].cmd());
      }
      
      swerve.field.getObject("autoTrajectory").setPoses(fullTrajectory);
    }
    
    return routine;  
  }
}
