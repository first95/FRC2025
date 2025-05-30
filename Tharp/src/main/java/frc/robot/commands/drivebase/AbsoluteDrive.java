// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

/** An example command that uses an example subsystem. */
public class AbsoluteDrive extends Command {
  private final SwerveBase swerve;
  private final PIDController thetaController;
  private final DoubleSupplier vX, vY, headingHorizontal, headingVertical;
  private final BooleanSupplier brakeButton;
  private boolean brake = false;
  private double omega, angle, lastAngle, x, y, xMoment, yMoment, zMoment,
    armHeight, armExtension; //maxAngularVelocity, maxAngularAccel;
  private final boolean isOpenLoop;
  private Translation2d horizontalCG;
  private Translation3d robotCG;
  private int debugFlags;

  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply 
   * translation inputs, where x is torwards/away from alliance wall and y is left/right.
   * headingHorizontal and headingVertical are the Cartesian coordinates from which the robot's angle
   * will be derived— they will be converted to a polar angle, which the robot will rotate to.
   *
   * @param swerve The swerve drivebase subsystem.
   * @param vX DoubleSupplier that supplies the x-translation joystick input.  Should
   * be in the range -1 to 1 with deadband already accounted for.  Positive X is away from the alliance
   * wall.
   * @param vY DoubleSupplier that supplies the y-translation joystick input.  Should be in the
   * range -1 to 1 with deadband already accounted for.  Positive Y is towards the left wall when looking
   * through the driver station glass.
   * @param headingHorizontal DoubleSupplier that supplies the horizontal component of the robot's heading angle.
   * In the robot coordinate system, this is along the same axis as vY. Should range from -1 to 1 with no 
   * deadband.  Positive is towards the left wall when looking through the driver station glass.
   * @param headingVertical DoubleSupplier that supplies the vertical component of the robot's heading angle.  In the
   * robot coordinate system, this is along the same axis as vX.  Should range from -1 to 1 with no deadband.
   * Positive is away from the alliance wall.
   */
  public AbsoluteDrive(SwerveBase swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingHorizontal, DoubleSupplier headingVertical, boolean isOpenLoop, BooleanSupplier brakeButton) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingHorizontal = headingHorizontal;
    this.headingVertical = headingVertical;
    this.isOpenLoop = isOpenLoop;
    this.brakeButton = brakeButton;
    thetaController = new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    
    addRequirements(swerve);
  }
  public void setHeading(Rotation2d heading) {
    lastAngle = heading.getRadians();
  }

  @Override
  public void initialize() {
    lastAngle = swerve.getTelePose().getRotation().getRadians();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    swerve.drive(new Translation2d(), 0, true, false);
    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
    // Checks if the gyro was reset, and, if so, sets the commanded heading to zero.
    // This allows the field refrence frame (which way is away from the alliance wall) to be
    // reset without the robot immediately rotating to the previously-commanded angle in the new
    // refrence frame.  This currently does not override the joystick.
    if (swerve.wasGyroReset()) {
      lastAngle = swerve.getTelePose().getRotation().getRadians();
      swerve.clearGyroReset();
    }

    if (brakeButton.getAsBoolean() || brake) {
      swerve.setDriveBrake();
    } else {

      // Converts the horizontal and vertical components to the commanded angle, in radians, unless
      // the joystick is near the center (i. e. has been released), in which case the angle is held
      // at the last valid joystick input (hold position when stick released).
      if (Math.hypot(headingHorizontal.getAsDouble(), headingVertical.getAsDouble()) < 0.5) {
        angle = lastAngle;
      } else {
        angle = Math.atan2(headingHorizontal.getAsDouble(), headingVertical.getAsDouble());
      }

      // Get the position of the arm from NetworkTables 
      armHeight = SmartDashboard.getNumber("armHeight", 0);
      armExtension = SmartDashboard.getNumber("armExtension", 0);

      xMoment = (armExtension * Constants.MANIPULATOR_MASS) + (Constants.CHASSIS_CG.getX() * Constants.CHASSIS_MASS);
      yMoment = (Constants.ARM_Y_POS * Constants.MANIPULATOR_MASS) + (Constants.CHASSIS_CG.getY() * Constants.CHASSIS_MASS);
      // Calculate the vertical mass moment using the floor as the datum.  This will be used later to calculate max acceleration
      zMoment = 
        (armHeight * Constants.MANIPULATOR_MASS)
        + (Constants.CHASSIS_CG.getZ() * (Constants.CHASSIS_MASS));
      
      robotCG = new Translation3d(xMoment, yMoment, zMoment).div(Constants.ROBOT_MASS);
      horizontalCG = robotCG.toTranslation2d();

      // Calculates an angular rate using a PIDController and the commanded angle.;
      omega = thetaController.calculate(swerve.getTelePose().getRotation().getRadians(), angle);
      omega = (Math.abs(omega) < Drivebase.HEADING_MIN_ANGULAR_CONTROL_EFFORT) ? 0 : omega;

      // Convert joystick inputs to m/s by scaling by max linear speed.  Also uses a cubic function
      // to allow for precise control and fast movement.
      x = Math.pow(vX.getAsDouble(), 3) * Drivebase.MAX_SPEED;
      y = Math.pow(vY.getAsDouble(), 3) * Drivebase.MAX_SPEED;

      // Limit velocity to prevent tippy
      //Translation2d translation = limitVelocity(new Translation2d(x, y));
      Translation2d translation = new Translation2d(x, y);
      

      // Make the robot move
      swerve.drive(translation, omega, true, isOpenLoop);

      SmartDashboard.putNumber("HeadingSetpoint", angle);
      SmartDashboard.putNumber("CommandedOmega", omega);
      SmartDashboard.putNumber("currentHeading", swerve.getTelePose().getRotation().getRadians());
      SmartDashboard.putNumber("DriveInputX", translation.getX());
      SmartDashboard.putNumber("DriveInputY", translation.getY());

      
    }
    
    // Used for the position hold feature
    lastAngle = angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
  public void setBrake(boolean setBrake){
    brake = setBrake;
  }
  /**
   * Calculates the maximum acceleration allowed in a direction without tipping the robot.
   * Reads arm position from NetworkTables and is passed the direction in question.
   * @param angle The direction in which to calculate max acceleration, as a Rotation2d.
   * Note that this is robot-relative.
   * @return
   */
  private double calcMaxAccel(Rotation2d angle) {
    

    // Determine the angle from the CG to each corner
    Rotation2d thetaFL = new Rotation2d(Drivebase.FRONT_LEFT_X - horizontalCG.getX(), Drivebase.FRONT_LEFT_Y - horizontalCG.getY());
    Rotation2d thetaFR = new Rotation2d(Drivebase.FRONT_RIGHT_X - horizontalCG.getX(), Drivebase.FRONT_RIGHT_Y - horizontalCG.getY());
    Rotation2d thetaBL = new Rotation2d(Drivebase.BACK_LEFT_X - horizontalCG.getX(), Drivebase.BACK_LEFT_Y - horizontalCG.getY());
    Rotation2d thetaBR = new Rotation2d(Drivebase.BACK_RIGHT_X - horizontalCG.getX(), Drivebase.BACK_RIGHT_Y - horizontalCG.getY());

    Rotation2d reverseAngle = angle.plus(Rotation2d.fromDegrees(180));
    Translation2d wheelbaseEdge;
    if (thetaFR.getRadians() <= reverseAngle.getRadians() && reverseAngle.getRadians() < thetaFL.getRadians()) {
      wheelbaseEdge = new Translation2d(
        Drivebase.FRONT_LEFT_X,
        reverseAngle.getTan() * (Drivebase.FRONT_LEFT_X - robotCG.getX()) + robotCG.getY()
      );
    } else if (thetaBL.getRadians() <= reverseAngle.getRadians() || reverseAngle.getRadians() <= thetaBR.getRadians()) {
      wheelbaseEdge = new Translation2d(
        Drivebase.BACK_LEFT_X,
        reverseAngle.getTan() * (Drivebase.BACK_LEFT_X - robotCG.getX()) + robotCG.getY()
      );
    } else if (thetaFL.getRadians() <= reverseAngle.getRadians() && reverseAngle.getRadians() < thetaBL.getRadians()) {
      wheelbaseEdge = new Translation2d(
        ((Drivebase.FRONT_LEFT_Y - robotCG.getY()) / reverseAngle.getTan()) + robotCG.getX(),
        Drivebase.FRONT_LEFT_Y
      );
    } else if (thetaBR.getRadians() <= reverseAngle.getRadians() && reverseAngle.getRadians() < thetaFR.getRadians()) {
      wheelbaseEdge = new Translation2d(
        ((Drivebase.FRONT_RIGHT_Y - robotCG.getY()) / reverseAngle.getTan()) + robotCG.getX(),
        Drivebase.FRONT_RIGHT_Y
      );
    } else {
      // This should never happen, just to make the compiler happy
      wheelbaseEdge = new Translation2d();
    }
    double distance = horizontalCG.minus(wheelbaseEdge).getNorm();
    return distance * Constants.GRAVITY / robotCG.getZ();
  }

  /**
   * Limits a commanded velocity to prevent exceeding the maximum acceleration given by 
   * {@link AbsoluteDrive#calcMaxAccel(Rotation2d)}.  Note that this takes and returns field-relative
   * velocities.
   * @param commandedVelocity The desired velocity
   * @return The limited velocity.  This is either the commanded velocity, if attainable, or the closest
   * attainable velocity.
   */
  private Translation2d limitVelocity(Translation2d commandedVelocity) {
    // Get the robot's current field-relative velocity
    ChassisSpeeds vel = swerve.getTeleFieldVelocity();
    Translation2d currentVelocity = new Translation2d(
        vel.vxMetersPerSecond,
        vel.vyMetersPerSecond);

    // Calculate the commanded change in velocity by subtracting current velocity
    // from commanded velocity
    Translation2d deltaV = commandedVelocity.minus(currentVelocity);
    SmartDashboard.putNumber("deltaV", deltaV.getX());

    // Creates an acceleration vector with the direction of delta V and a magnitude
    // of the maximum allowed acceleration in that direction 
    Translation2d maxAccel = new Translation2d(
      calcMaxAccel(deltaV
        // Rotates the velocity vector to convert from field-relative to robot-relative
        .rotateBy(swerve.getTelePose().getRotation().unaryMinus())
        .getAngle()),
      deltaV.getAngle());

    // Calculate the maximum achievable velocity by the next loop cycle.
    // delta V = Vf - Vi = at
    Translation2d maxAchievableDeltaVelocity = maxAccel.times(Constants.SPARK_VELOCITY_RESPONSE_LOOP);
    
    if (deltaV.getNorm() > maxAchievableDeltaVelocity.getNorm()) {
      return maxAchievableDeltaVelocity.plus(currentVelocity);
    } else {
      // If the commanded velocity is attainable, use that.
      return commandedVelocity;
    }
  }
}
