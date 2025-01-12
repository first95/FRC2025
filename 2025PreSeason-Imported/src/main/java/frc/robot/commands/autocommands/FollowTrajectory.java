// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
  private final PIDController xController = new PIDController(Auton.DRIVE_KP, Auton.DRIVE_KI, Auton.DRIVE_KD);
  private final PIDController yController = new PIDController(Auton.DRIVE_KP, Auton.DRIVE_KI, Auton.DRIVE_KD);
  private final PIDController headingController = new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);

  public FollowTrajectory(SwerveBase swerve, SwerveSample sample) {
    
    Pose2d pose = swerve.getPose();

    ChassisSpeeds speeds = new ChassisSpeeds(
      sample.vx + xController.calculate(pose.getX(), sample.x),
      sample.vy + yController.calculate(pose.getY(), sample.y),
      sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
    );
    swerve.setChassisSpeeds(speeds);
   
  }
}
