// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.CamData;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SwerveModule;
import frc.robot.Constants.Auton;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.Vision;

import choreo.trajectory.SwerveSample;
import choreo.auto.AutoFactory;


import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveBase extends SubsystemBase {
  private final PIDController autonXController = new PIDController(Auton.FOLLOW_TRAJECTORY_KP, Auton.FOLLOW_TRAJECTORY_KI, Auton.FOLLOW_TRAJECTORY_KD);
  private final PIDController autonYController = new PIDController(Auton.FOLLOW_TRAJECTORY_KP, Auton.FOLLOW_TRAJECTORY_KI, Auton.FOLLOW_TRAJECTORY_KD);
  private final PIDController autonHeadingController = new PIDController(Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
  private SwerveSample autonSetpoint;
  

  private final SwerveModule[] swerveModules;
  private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[Drivebase.NUM_MODULES];
  private SwerveModuleState[] currentModuleStates = new SwerveModuleState[Drivebase.NUM_MODULES];
  private Pigeon2 imu;
  
  public Field2d field = new Field2d();
  private Field2d sternCam, bowCam;

  private double angle, lasttime;

  private double[] moduleStates = new double[8];
  private double[] moduleSetpoints = new double[8];

  private int poseErrorCounter, debugFlags;

  private Timer timer;

  private boolean wasGyroReset, wasOdometrySeeded;

  private final SwerveDrivePoseEstimator odometry;

  private Pose2d currentPose, currentTelePose;

  private ChassisSpeeds currentRobotVelocity, currentFieldVelocity, currentTeleFieldVelocity;

  private Alliance alliance = null;

  public boolean isAuto;

  private final SysIdRoutine driveCharacterizer, angleCharacterizer;

  /** Creates a new swerve drivebase subsystem.  Robot is controlled via the drive() method,
   * or via the setModuleStates() method.  The drive() method incorporates kinematics— it takes a 
   * translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * setModuleStates() takes a list of SwerveModuleStates and directly passes them to the modules.
   * This subsytem also handles odometry.
  */
  public SwerveBase() {
    

    // Create an integrator for angle if the robot is being simulated to emulate an IMU
    // If the robot is real, instantiate the IMU instead.
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lasttime = 0;
    } else {
      imu = new Pigeon2(Drivebase.PIGEON);
      Pigeon2Configurator configurator = imu.getConfigurator();
      Pigeon2Configuration config = new Pigeon2Configuration();
      config.MountPose.MountPosePitch = Drivebase.IMU_MOUNT_PITCH;
      config.MountPose.MountPoseRoll = Drivebase.IMU_MOUNT_ROLL;
      config.MountPose.MountPoseYaw = Drivebase.IMU_MOUNT_YAW;
      configurator.apply(config);
    }

    this.swerveModules = new SwerveModule[] {
      new SwerveModule(Drivebase.Mod0.CONSTANTS),
      new SwerveModule(Drivebase.Mod1.CONSTANTS),
      new SwerveModule(Drivebase.Mod2.CONSTANTS),
      new SwerveModule(Drivebase.Mod3.CONSTANTS),
    };
    for (SwerveModule module : swerveModules) {
      currentModulePositions[module.moduleNumber] = module.getPosition();
    }
    
    odometry = new SwerveDrivePoseEstimator(
      Drivebase.KINEMATICS,
      getYaw(),
      currentModulePositions,
      new Pose2d(),
      VecBuilder.fill(Vision.ODOMETRY_TRANSLATIONAL_STD_DEV, Vision.ODOMETRY_TRANSLATIONAL_STD_DEV, Vision.ODOMETRY_ANGULAR_STD_DEV),
      VecBuilder.fill(0.01, 0.01, 0.02));
    wasOdometrySeeded = false;
    wasGyroReset = false;
    currentPose = new Pose2d();
    currentTelePose = new Pose2d();

    currentRobotVelocity = new ChassisSpeeds();
    currentFieldVelocity = new ChassisSpeeds();
    currentTeleFieldVelocity = new ChassisSpeeds();

    sternCam = new Field2d();
    bowCam = new Field2d();
    SmartDashboard.putData("SternCam", sternCam);
    SmartDashboard.putData("BowCam", bowCam);
    SmartDashboard.putData("Field", field);
    poseErrorCounter = 0;

    debugFlags = (int)SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    driveCharacterizer = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
          for (SwerveModule module : this.swerveModules) {
            module.setAzimuth(new Rotation2d());
            module.setDriveVolts(volts.in(Volts));
          }
        },
        log -> {
          log.motor("driveLinear")
          .voltage(Volts.of(avgDriveVolts()))
          .linearPosition(Meters.of(odometry.getEstimatedPosition().getX()))
          .linearVelocity(MetersPerSecond.of(getRobotVelocity().vxMetersPerSecond));
        },
        this));
    
    angleCharacterizer = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Voltage volts) -> {
          for (SwerveModule module : this.swerveModules) {
            module.setAzimuth(module.getModuleLocation().getAngle().plus(Rotation2d.fromDegrees(90)));
            module.setDriveVolts(volts.in(Volts));
          }
        },
        log -> {
          log.motor("driveAngular")
          .voltage(Volts.of(avgDriveVolts()))
          .angularPosition(Radians.of(currentPose.getRotation().getRadians()))
          .angularVelocity(RadiansPerSecond.of(getRobotVelocity().omegaRadiansPerSecond));
        },
        this));

    autonHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    autonXController.setTolerance(Constants.Auton.DRIVE_POSITIONAL_TOLERANCE);
    autonYController.setTolerance(Constants.Auton.DRIVE_POSITIONAL_TOLERANCE);
    autonHeadingController.setTolerance(Constants.Drivebase.HEADING_TOLERANCE);
  }

  /**
   * The primary method for controlling the drivebase.  Takes a Translation2d and a rotation rate, and 
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity
   * control for the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation
   *  vector is used.
   * !!!IMPORTANT!!! -- this method flips input speeds when on the Red alliance.  Use setChassisSpeeds for auton.
   * @param translation  Translation2d that is the commanded linear velocity of the robot, in meters per second.
   * In robot-relative mode, positive x is torwards the bow (front) and positive y is torwards port (left).  In field-
   * relative mode, positive x is away from the alliance wall (field North) and positive y is torwards the left wall when looking 
   * through the driver station glass (field West).
   * @param rotation  Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot relativity.
   * @param fieldRelative  Drive mode.  True for field-relative, false for robot-relative.
   * @param isOpenLoop  Whether or not to use closed-loop velocity control.  Set to true to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if necessary.
    ChassisSpeeds velocity = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
      correctForDynamics(
        new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation),
        Constants.LOOP_CYCLE,
        Drivebase.SKEW_CORRECTION_FACTOR),
      currentTelePose.getRotation()
    )
    : new ChassisSpeeds(
      translation.getX(),
      translation.getY(),
      rotation
    );

    // Display commanded speed for testing
    if ((debugFlags & Drivebase.DEBUG_FLAG) != 0) {
      SmartDashboard.putString("InputCommands", translation.toString() + ", Omega: " + rotation);
      SmartDashboard.putString("DesiredRobotVelocity", velocity.toString());
    }

    // Calculate required module states via kinematics
    SwerveModuleState[] swerveModuleStates = 
      Drivebase.KINEMATICS.toSwerveModuleStates(
        velocity
      );
    
    // Desaturate calculated speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Drivebase.MAX_SPEED);

    // Command and display desired states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly.  Used primarily for auto
   * pathing.
   * @param desiredStates  A list of SwerveModuleStates to send to the modules.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Desaturates wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drivebase.MAX_SPEED);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  /**
   * Set robot-relative chassis speeds with closed-loop velocity control.
   * @param chassisSpeeds Robot-relative.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      Drivebase.KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  public void setFieldRelChassisSpeedsAndSkewCorrect(ChassisSpeeds chassisSpeeds) {
    setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
      correctForDynamics(
        chassisSpeeds,
        Constants.LOOP_CYCLE,
        Drivebase.SKEW_CORRECTION_FACTOR),
      currentPose.getRotation()));
  }

  public void followTrajectory(SwerveSample sample) {
    
    Pose2d pose = getPose();
    autonSetpoint = sample;

    ChassisSpeeds speeds = new ChassisSpeeds(
      sample.vx + autonXController.calculate(pose.getX(), sample.x),
      sample.vy + autonYController.calculate(pose.getY(), sample.y),
      sample.omega + autonHeadingController.calculate(pose.getRotation().getRadians(), sample.heading)
    );
    

    setFieldRelChassisSpeedsAndSkewCorrect(speeds);
   
  }

  
  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return currentPose;
  }

  public Pose2d getTelePose() {
    return currentTelePose;
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return currentFieldVelocity;
  }

  public ChassisSpeeds getTeleFieldVelocity() {
    return currentTeleFieldVelocity;
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return currentRobotVelocity;
  }

  /**
   * Gets the current module states (azimuth and velocity)
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState[] getStates() {
    return currentModuleStates;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   * @return A list of SwerveModulePositions cointaing the current module positions
   */
  public SwerveModulePosition[] getCurrentModulePositions() {
    return currentModulePositions;
  }
  
  public boolean wasGyroReset() {
    return wasGyroReset;
  }

  public void clearGyroReset() {
    wasGyroReset = false;
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   * @return The yaw angle
   */
  private Rotation2d getYaw() {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (Robot.isReal()) {
      double yaw = imu.getYaw().getValueAsDouble();
      return (Drivebase.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    } else {
      return new Rotation2d(angle);
    }
  }

  public Rotation2d getPitch() {
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(imu.getPitch().getValueAsDouble());
    } else {
      return new Rotation2d();
    }
  }

  public Rotation2d getRoll() {
    if (Robot.isReal()) {
      return Rotation2d.fromDegrees(imu.getRoll().getValueAsDouble());
    } else {
      return new Rotation2d();
    }
  }

  public Command sysIdQuasiLinear(SysIdRoutine.Direction direction) {
    return driveCharacterizer.quasistatic(direction);
  }
  public Command sysIdQuasiAngular(SysIdRoutine.Direction direction) {
    return angleCharacterizer.quasistatic(direction);
  }
  public Command sysIdDynLinear(SysIdRoutine.Direction direction) {
    return driveCharacterizer.dynamic(direction);
  }
  public Command sysIdDynAngular(SysIdRoutine.Direction direction) {
    return angleCharacterizer.dynamic(direction);
  }

  /**
   * Sets the drive motors to brake/coast mode.
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
        new SwerveModuleState(
          0,
          Drivebase.MODULE_LOCATIONS[swerveModule.moduleNumber].getAngle()),
        true,
        false);
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public Alliance getAlliance() {
    return alliance;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getCurrentModulePositions(), pose);
  }

  public void clearOdometrySeed() {
    wasOdometrySeeded = false;
  }

  public void setVelocityModuleGains() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setVelocityGains(
        SmartDashboard.getNumber("KP", Drivebase.VELOCITY_KP),
        SmartDashboard.getNumber("KI", Drivebase.VELOCITY_KI),
        SmartDashboard.getNumber("KD", Drivebase.VELOCITY_KD),
        SmartDashboard.getNumber("KS", Drivebase.KS),
        SmartDashboard.getNumber("KV", Drivebase.KV),
        SmartDashboard.getNumber("KA", Drivebase.KA));
    }
  }
  
  private CamData addVisionMeasurement(String limelightName, Pose2d estimatedPose) {
    return addVisionMeasurement(limelightName, estimatedPose, 1);
  }

  private CamData addVisionMeasurement(String limelightName, Pose2d estimatedPose, int minTags) {
    CamData visionMeasurement = new CamData(limelightName);

    //double poseDifference = estimatedPose.getTranslation().getDistance(visionMeasurement.pose2d.getTranslation());
    double angDifference = estimatedPose.getRotation().minus(visionMeasurement.pose2d.getRotation()).getRadians();
    double xyStds, angStds;
    boolean useTagTheta = false;

    if (!visionMeasurement.valid
    || visionMeasurement.pipeline != Vision.APRILTAG_PIPELINE_NUMBER
    || (Math.abs(visionMeasurement.pose3d.getZ()) > Vision.MAX_ALLOWABLE_Z_ERROR)
    || (Math.abs(angDifference) > Vision.ANGULAR_ERROR_TOLERANCE)
    || (visionMeasurement.numTargets < minTags)) {
      if ((debugFlags & Vision.DEBUG_FLAG) != 0) {
        SmartDashboard.putBoolean(limelightName + " Tests", false);
      }
      return visionMeasurement;
    }
    /*if ((poseDifference > Vision.POSE_ERROR_TOLERANCE) && (poseErrorCounter < Vision.LOOP_CYCLES_BEFORE_RESET)) {
      poseErrorCounter++;
      if ((debugFlags & Vision.DEBUG_FLAG) != 0) {
        SmartDashboard.putBoolean(limelightName + " Tests", false);
      }
      return visionMeasurement;
    } else if (poseErrorCounter >= Vision.LOOP_CYCLES_BEFORE_RESET) {
      wasOdometrySeeded = false;
      return visionMeasurement;
    }
    poseErrorCounter = 0;*/
    
    if (visionMeasurement.numTargets > 1) {
      useTagTheta = true;
    }

    xyStds = Vision.XY_STD_DEV_COEFFICIENT * Math.pow(visionMeasurement.avgTagDist, 2) / visionMeasurement.numTargets;
    angStds = useTagTheta ? Vision.ANG_STD_DEV_COEFFICIENT * Math.pow(visionMeasurement.avgTagDist, 2) / visionMeasurement.numTargets : Double.POSITIVE_INFINITY;

    if ((debugFlags & Vision.DEBUG_FLAG) != 0) {
        SmartDashboard.putBoolean(limelightName + " Tests", true);
        SmartDashboard.putNumber(limelightName + "XyStdDev", xyStds);
        SmartDashboard.putNumber(limelightName + "AngStds", angStds);
        }
    double timestamp = Timer.getFPGATimestamp() - (visionMeasurement.latency / 1000);
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, angStds));
    odometry.addVisionMeasurement(visionMeasurement.pose2d, timestamp);

    return visionMeasurement;
  }

  @Override
  public void periodic() {
    // Read active flags
    debugFlags = (int)SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
    

    // Update odometry and current pose/velocity
    for (SwerveModule module : swerveModules) {
      currentModulePositions[module.moduleNumber] = module.getPosition();
      currentModuleStates[module.moduleNumber] = module.getState();

        SmartDashboard.putNumber("Module " + module.moduleNumber + " Absolute Encoder", module.getAbsoluteEncoder());
        SmartDashboard.putNumber("Module " + module.moduleNumber + " Speed", module.getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + module.moduleNumber + " Drive Current", module.getDriveCurrent());
        SmartDashboard.putNumber("Module " + module.moduleNumber + " Drive Voltage", module.getDriveVolts());
        
        var desState = module.getDesiredState();
        SmartDashboard.putNumber("Module " + module.moduleNumber + " Target Angle", desState.angle.getDegrees());
        SmartDashboard.putNumber("Module " + module.moduleNumber + " Target Speed", desState.speedMetersPerSecond);
        moduleStates[module.moduleNumber] = currentModuleStates[module.moduleNumber].angle.getRadians();
        moduleSetpoints[module.moduleNumber] = desState.angle.getRadians();
        moduleStates[module.moduleNumber + 1] = currentModuleStates[module.moduleNumber].speedMetersPerSecond;
        moduleSetpoints[module.moduleNumber + 1] = desState.speedMetersPerSecond;
    }
    odometry.update(getYaw(), currentModulePositions);
    currentPose = odometry.getEstimatedPosition();
    currentTelePose = alliance == Alliance.Red 
    ? new Pose2d(currentPose.getTranslation(), currentPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)))
    : currentPose;
    currentRobotVelocity = Drivebase.KINEMATICS.toChassisSpeeds(currentModuleStates);
    currentFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(currentRobotVelocity, currentPose.getRotation());
    currentTeleFieldVelocity = alliance == Alliance.Red
    ? new ChassisSpeeds(-currentFieldVelocity.vxMetersPerSecond, -currentFieldVelocity.vyMetersPerSecond, currentFieldVelocity.omegaRadiansPerSecond)
    : currentFieldVelocity;

    SmartDashboard.putString("alliance", (alliance != null) ? alliance.toString() : "NULL");
    SmartDashboard.putString("OdometryPos", currentPose.toString());
    SmartDashboard.putBoolean("seeded", wasOdometrySeeded);
    
    // Seed odometry if this has not been done
    if (!wasOdometrySeeded && alliance != null) { 
      CamData bowSeed = new CamData(Vision.BOW_LIMELIGHT_NAME);
      CamData sternSeed = new CamData(Vision.STERN_LIMELIGHT_NAME);
      if ((bowSeed.valid) && (sternSeed.valid)) {
        // Average poses together
        Translation2d translation = bowSeed.pose2d.getTranslation().plus(sternSeed.pose2d.getTranslation()).div(2);
        // Rotation average:
        Rotation2d rotation = 
          new Translation2d(
            (bowSeed.pose2d.getRotation().getCos() + sternSeed.pose2d.getRotation().getCos()) / 2,
            (bowSeed.pose2d.getRotation().getSin() + sternSeed.pose2d.getRotation().getSin()) / 2)
          .getAngle();
        resetOdometry(new Pose2d(translation, rotation));
        wasOdometrySeeded = true;
        wasGyroReset = true;
      }
      else if (sternSeed.valid) {
        resetOdometry(sternSeed.pose2d);
        wasOdometrySeeded = true;
        wasGyroReset = true;
      }
      else if (bowSeed.valid) {
        resetOdometry(bowSeed.pose2d);
        wasOdometrySeeded = true;
        wasGyroReset = true;
      } else {
        DriverStation.reportWarning("No tag visible! Odometry not seeded!", false);
      }
    } else if (alliance == null) {
      DriverStation.reportError("Alliance not set!!  Odometry not seeded!", false);
    }
    
    CamData sternCamPose = addVisionMeasurement(Vision.STERN_LIMELIGHT_NAME, currentPose);
    CamData bowCamPose = addVisionMeasurement(Vision.BOW_LIMELIGHT_NAME, currentPose, sternCamPose.numTargets);

    field.setRobotPose(currentPose);
    bowCam.setRobotPose(bowCamPose.pose2d);
    sternCam.setRobotPose(sternCamPose.pose2d);

    // Update angle accumulator if the robot is simulated
    if (!Robot.isReal()) {
      angle += Drivebase.KINEMATICS.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lasttime);
      lasttime = timer.get();
      
    }

    if ((debugFlags & Vision.DEBUG_FLAG) != 0) {
      SmartDashboard.putNumber("BowCamZ", bowCamPose.pose3d.getZ());
      SmartDashboard.putBoolean("BowValid", bowCamPose.valid);
      SmartDashboard.putNumber("BowTa", bowCamPose.ta);
      SmartDashboard.putNumber("BowNumTarg", bowCamPose.numTargets);
      SmartDashboard.putNumber("BowPipe", bowCamPose.pipeline);
      SmartDashboard.putNumber("BowLatency", bowCamPose.latency);

      SmartDashboard.putNumber("SternCamZ", sternCamPose.pose3d.getZ());
      SmartDashboard.putBoolean("SternValid", sternCamPose.valid);
      SmartDashboard.putNumber("SternTa", sternCamPose.ta);
      SmartDashboard.putNumber("SternNumTarg", sternCamPose.numTargets);
      SmartDashboard.putNumber("SternPipe", sternCamPose.pipeline);
      SmartDashboard.putNumber("SternLatency", sternCamPose.latency);

      SmartDashboard.putNumber("PoseErrorCounter", poseErrorCounter);
    }
    if ((debugFlags & Drivebase.DEBUG_FLAG) != 0) {
      SmartDashboard.putNumberArray("moduleStates", moduleStates);
      SmartDashboard.putNumberArray("moduleSetpoints", moduleSetpoints);
      SmartDashboard.putNumber("OdomHeading", currentPose.getRotation().getRadians());
      SmartDashboard.putString("Gyro", getYaw().toString());
      SmartDashboard.putNumber("Robot X Vel", currentRobotVelocity.vxMetersPerSecond);
      SmartDashboard.putNumber("Robot Y Vel", currentRobotVelocity.vyMetersPerSecond);
      SmartDashboard.putNumber("Robot Ang Vel", currentRobotVelocity.omegaRadiansPerSecond);
    }
    if(autonSetpoint != null){
      SmartDashboard.putNumber("AutonXSetpoint", autonSetpoint.x);
      SmartDashboard.putNumber("AutonYSetpoint", autonSetpoint.y);
      SmartDashboard.putNumber("AutonHeadingSetpoint", autonSetpoint.heading);
      field.getObject("AutonPoseSetPoint").setPose(autonSetpoint.getPose());
      
    } 
    SmartDashboard.putNumber("CurrentRobotX", currentPose.getX());
    SmartDashboard.putNumber("CurrentRobotY", currentPose.getY());
    SmartDashboard.putNumber("CurrentRobotHeading", currentPose.getRotation().getRadians());
  }

  @Override
  public void simulationPeriodic() {
  }


  private ChassisSpeeds correctForDynamics(ChassisSpeeds initial, double dt, double magicFactor) {
    final double oneMinusCos = 1 - Math.cos(initial.omegaRadiansPerSecond * dt);
    if (Math.abs(oneMinusCos) < 1E-9) {
      return initial;
    } else {
      final var linearVel = new Translation2d(initial.vxMetersPerSecond, initial.vyMetersPerSecond);
      final double tangentVel = linearVel.getNorm();
      final double radius;
      radius = tangentVel / initial.omegaRadiansPerSecond;
      final double skewVelocity = (radius * oneMinusCos) / dt;
      var direction = linearVel.getAngle().minus(Rotation2d.fromDegrees(90));
      var velocityCorrection = new Translation2d(skewVelocity, direction).times(magicFactor);
      var translationVel = linearVel.plus(velocityCorrection);
      return new ChassisSpeeds(
        translationVel.getX(),
        translationVel.getY(),
        initial.omegaRadiansPerSecond
      );
    }
  }
  
  public void turnModules(double speed) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.turnModule(speed);
    }
  }

  private double avgDriveVolts() {
    double sum = 0;
    for (SwerveModule module : swerveModules) {
      sum += module.getDriveVolts();
    }
    return (sum / Drivebase.NUM_MODULES);
  }

}
