// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.SwerveBase;

/** An example command that uses an example subsystem. */
public class AutoShoot extends Command {
  private final PIDController thetaController;
  private final SwerveBase swerve;
  private final double timeout;
  private final Timer timer;

  private double angle, omega;
  private Pose2d currentPose;
  private Translation3d speakerLocation;
  private int debugFlags;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShoot(SwerveBase swerve) {
    this(swerve, -1);
  }
  public AutoShoot(SwerveBase swerve, double timeoutSeconds) {
    thetaController = new PIDController(
      Drivebase.HEADING_KP,
      Drivebase.HEADING_KI,
      Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Drivebase.HEADING_TOLERANCE);
    this.swerve = swerve;
    timeout = timeoutSeconds;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean(Auton.ON_TARGET_KEY, false);

    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    if (timeout >= 0) {
      timer.reset();
      timer.start();
    }

    var alliance = swerve.getAlliance();
    if (alliance == Alliance.Blue) {
      speakerLocation = Vision.BLUE_SPEAKER_POS;
    } else if (alliance == Alliance.Red) {
      speakerLocation = Vision.RED_SPEAKER_POS;
    } else {
      DriverStation.reportError("AutoShoot aborted because no alliance is set!", false);
      cancel();
    }

    currentPose = swerve.getPose();

    var centerPosDelta = speakerLocation.minus(new Translation3d(currentPose.getX(), currentPose.getY(), speakerLocation.getZ()));

    // Calculate azimuth using position of center of robot:
    // Azimuth:
    // Note that we shoot from the back of the robot, so it may seem like this is backwards what it should be.
    angle = new Rotation2d(centerPosDelta.getX(), centerPosDelta.getY())
      .plus(Auton.AUTO_SHOOT_AZIMUTH_ADJUSTMENT)
      .plus(Rotation2d.fromDegrees(180)).getRadians();

    var shoulderLocation = 
      // package current pose into translation3d
      new Translation3d(currentPose.getX(), currentPose.getY(), 0)
      // add mount location of arm, after converting to field-relative coordinates
      .plus(
        ArmConstants.PIVOT_LOCATION.rotateBy(
          new Rotation3d(0, 0, -angle)
        )
      );  
    var posDelta = speakerLocation.minus(shoulderLocation);

    // Elevation (now with gravity!)
    var range = posDelta.toTranslation2d().getNorm();
    var height = posDelta.getZ();

    if (range > ShooterConstants.AUTO_SHOOT_MAX_RANGE) {
      DriverStation.reportError("AutoShoot aborted! Outside maximum range!", false);
      cancel();
    }

    // Linearly scale speed to account for drag— we only care about average speed (drop => time-of-flight => avg. speed)
    // so a linear interpolation is fine.
    var speed = range * ShooterConstants.DRAG_INTERPOLATION_SLOPE + ShooterConstants.DRAG_INTERPOLATION_INTERCEPT;

    // Coefficients for quadratic to do physics stuff
    var A = -(Constants.GRAVITY * Math.pow(range, 2)) / (2 * Math.pow(speed, 2));
    var B = range;
    var C = A - height;
    // Quadratic formula; simulating in desmos shows the addition root is the one we want
    var root = (-B + Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
    // Solving the quadratic gives the tangent of the angle
    double elevation = Math.atan(root);

    if (Double.isNaN(elevation) || (elevation > (Math.PI) / 2)) {
      DriverStation.reportError("AutoShoot aborted!  Invalid solution!", false);
      cancel();
    }
    if (elevation < ArmConstants.LOWER_LIMIT.getRadians()) {
      elevation = ArmConstants.LOWER_LIMIT.getRadians() + ShooterConstants.DEAD_ZONE_FUDGE_OFFSET.getRadians();
    }

    SmartDashboard.putNumber(Auton.ARM_ANGLE_KEY, elevation);

    thetaController.reset();

    SmartDashboard.putBoolean(Auton.AUTO_SHOOTING_KEY, true);

    if ((debugFlags & CommandDebugFlags.AUTO_SHOOT) != 0) {
      SmartDashboard.putNumber("AutoShootHeading", angle);
      SmartDashboard.putNumber("AutoShootRange", range);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    omega = thetaController.calculate(swerve.getPose().getRotation().getRadians(), angle);
    omega = (Math.abs(omega) < Drivebase.HEADING_MIN_ANGULAR_CONTROL_EFFORT) ? 0 : omega;
    swerve.drive(new Translation2d(), omega, true, false);

    if (thetaController.atSetpoint()) {
      SmartDashboard.putBoolean(Auton.ON_TARGET_KEY, true);
      swerve.setDriveBrake();
    }

    if ((debugFlags & CommandDebugFlags.AUTO_SHOOT) != 0) {
      // Put SmartDashboard debug here
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean(Auton.AUTO_SHOOTING_KEY, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeout < 0) {
      return false;
    } else {
      return timer.get() >= timeout;
    }
  }
}
