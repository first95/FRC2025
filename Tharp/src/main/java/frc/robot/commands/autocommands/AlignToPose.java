// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

public class AlignToPose extends Command {
  private Pose2d target, initialPose;
  private Supplier<Pose2d> targetSupplier;
  private String targetName;
  private final SwerveBase swerve;
  private final TrapezoidProfile driveProfile;
  private final PIDController xController, yController, thetaController;
  private final Timer timer;
  private final boolean stringPose;
  
  private Pose2d currentRelativePose, currentPose;
  private double initialDistance, cosine, sine;
  private TrapezoidProfile.State goalState, initialState;

  /** Creates a new AlignToPose. */
  public AlignToPose(Supplier<Pose2d> pose, SwerveBase swerve) {
    targetSupplier = pose;
    this.swerve = swerve;
    driveProfile = new TrapezoidProfile(Auton.DRIVE_CONSTRAINTS);
    xController = new PIDController(
      Auton.ALIGN_TO_POSE_KP, Auton.ALIGN_TO_POSE_KI, Auton.ALIGN_TO_POSE_KD);
    yController = new PIDController(
      Auton.ALIGN_TO_POSE_KP, Auton.ALIGN_TO_POSE_KI, Auton.ALIGN_TO_POSE_KD);
    thetaController = new PIDController(
      Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    timer = new Timer();
    stringPose = false;

    addRequirements(swerve);
  }
  public AlignToPose(Pose2d pose, SwerveBase swerve) {
    target = pose;
    this.swerve = swerve;
    driveProfile = new TrapezoidProfile(Auton.DRIVE_CONSTRAINTS);
    xController = new PIDController(
      Auton.ALIGN_TO_POSE_KP, Auton.ALIGN_TO_POSE_KI, Auton.ALIGN_TO_POSE_KD);
    yController = new PIDController(
      Auton.ALIGN_TO_POSE_KP, Auton.ALIGN_TO_POSE_KI, Auton.ALIGN_TO_POSE_KD);
    thetaController = new PIDController(
      Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    timer = new Timer();
    stringPose = false;

    addRequirements(swerve);
  }

  public AlignToPose(String poseName, SwerveBase swerve) {
    targetName = poseName;
    this.swerve = swerve;
    driveProfile = new TrapezoidProfile(Auton.DRIVE_CONSTRAINTS);
    xController = new PIDController(
      Auton.ALIGN_TO_POSE_KP, Auton.ALIGN_TO_POSE_KI, Auton.ALIGN_TO_POSE_KD);
    yController = new PIDController(
      Auton.ALIGN_TO_POSE_KP, Auton.ALIGN_TO_POSE_KI, Auton.ALIGN_TO_POSE_KD);
    thetaController = new PIDController(
      Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    timer = new Timer();
    stringPose = true;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    if (stringPose) {
      try {
        target = Auton.POSE_MAP.get(swerve.getAlliance()).get(targetName);
      } catch (NullPointerException e) {
        DriverStation.reportError("AlignToPose aborted! Alliance Invalid; could not fetch poses!", false);
        cancel();
      }
      if (target == null) {
        DriverStation.reportError("AlignToPose aborted! Named pose does not exist!", false);
        cancel();
      }
    }
    else if(targetSupplier != null){
      target = targetSupplier.get();
    }
    SmartDashboard.putNumber("AutoAlignTargetTheta", target.getRotation().getDegrees());
    initialPose = swerve.getPose();
    currentPose = initialPose;
    var deltaTranslation = target.getTranslation().minus(initialPose.getTranslation());
    var currentVelocity = swerve.getFieldVelocity();
    var driveAngle = deltaTranslation.getAngle();
    SmartDashboard.putNumber("AutoAlignDriveAngle", driveAngle.getDegrees());
    SmartDashboard.putNumber("AutoAlignTargetX", target.getX());
    SmartDashboard.putNumber("AutoAlignTargetY", target.getY());
    cosine = driveAngle.getCos();
    sine = driveAngle.getSin();
    initialDistance = deltaTranslation.getNorm();
    //fix initial velocity
    initialState = new TrapezoidProfile.State(0, Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond));
    goalState = new TrapezoidProfile.State(initialDistance, 0);
    SmartDashboard.putNumber("AutoAlignGoal", initialDistance);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = swerve.getPose();
    currentRelativePose = new Pose2d(
      currentPose.getTranslation().minus(initialPose.getTranslation()),
      currentPose.getRotation()
    );
    var profileSetpoint = driveProfile.calculate(timer.get(), initialState, goalState);
    SmartDashboard.putNumber("AutoAlignPosSetpoint", profileSetpoint.position);
    SmartDashboard.putNumber("AutoAlignVelSetpoint", profileSetpoint.velocity);
    
    var x = xController.calculate(
      currentRelativePose.getX(),
      profileSetpoint.position * cosine
    ) + profileSetpoint.velocity * cosine;
    var y = yController.calculate(
      currentRelativePose.getY(),
      profileSetpoint.position * sine
    ) + profileSetpoint.velocity * sine;
    SmartDashboard.putNumber("XPosSetpoint", profileSetpoint.position * cosine);
    SmartDashboard.putNumber("YPosSetpoint", profileSetpoint.position * sine);
    SmartDashboard.putNumber("XPos", currentPose.getX());
    SmartDashboard.putNumber("Ypos", currentPose.getY());

    var omega = thetaController.calculate(currentRelativePose.getRotation().getRadians(), target.getRotation().getRadians());
    omega = (Math.abs(omega) < Drivebase.HEADING_MIN_ANGULAR_CONTROL_EFFORT) ? 0 : omega;

    swerve.setFieldRelChassisSpeedsAndSkewCorrect(new ChassisSpeeds(x, y, omega));

    SmartDashboard.putBoolean("XYatTarg", currentPose.getTranslation().getDistance(target.getTranslation()) <= Auton.DRIVE_POSITIONAL_TOLERANCE);
    SmartDashboard.putBoolean("thetaAtTarg", currentPose.getRotation().minus(target.getRotation()).getRadians() <= Drivebase.HEADING_TOLERANCE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setDriveBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentPose.getTranslation().getDistance(target.getTranslation()) <= Auton.DRIVE_POSITIONAL_TOLERANCE)
    && currentPose.getRotation().minus(target.getRotation()).getRadians() <= Drivebase.HEADING_TOLERANCE;
  }
}
