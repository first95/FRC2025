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
import java.util.Map;

import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private final AutoFactory autoFactory;
  private final SwerveBase swerve;
  
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
  }
  public Command Diamond(){
    return Commands.sequence(
      autoFactory.resetOdometry("Diamond"),
      autoFactory.trajectoryCmd("Diamond"));
  }
  
  public AutoRoutine testModularAuto(String[] posTargets){
    AutoRoutine routine = autoFactory.newRoutine("TestModularAuto");
    if (posTargets.length > 2){
      AutoTrajectory[] trajectories = {};

      

      //load trajectorys based on posTargets
      for(int n = 0; n < trajectories.length; n++){
        trajectories[n] = routine.trajectory(posTargets[n] + " - " + posTargets[n+1]);
      } 
  
      routine.active().onTrue(
        Commands.sequence(
          trajectories[0].resetOdometry(),
          trajectories[0].cmd()
        )
      );
      
      for(int n = 0; n < trajectories.length; n++){
        trajectories[n].done().onTrue(trajectories[n+1].cmd());
      }
      
      
    }
    return routine;  
  }
}
