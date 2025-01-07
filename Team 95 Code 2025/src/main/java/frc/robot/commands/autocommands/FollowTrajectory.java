// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTrajectory extends SequentialCommandGroup {
  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(ChoreoTrajectory trajectory, SwerveBase swerve, boolean resetOdometry, boolean stopAtEnd) {
    if (resetOdometry) {
      addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())));
    }
    addCommands(
      Choreo.choreoSwerveCommand(
        trajectory,
        swerve::getPose,
        new PIDController(Auton.DRIVE_KP, Auton.DRIVE_KI, Auton.DRIVE_KD),
        new PIDController(Auton.DRIVE_KP, Auton.DRIVE_KI, Auton.DRIVE_KD),
        new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD),
        swerve::setChassisSpeeds,
        () -> swerve.getAlliance() == Alliance.Red,
        swerve)
    );
    if (stopAtEnd) {
      addCommands(new InstantCommand(() -> swerve.setChassisSpeeds(new ChassisSpeeds())));
    }
  }
}
