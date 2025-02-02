// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double NEO_FREE_SPEED = 5676; // RPM
    public static final double NEO_STALL_TORQUE = 3.75; // N * m
    public static final double NEO_550_FREE_SPEED = 11000; // RPM
  public static final double SPARK_VELOCITY_RESPONSE_LOOP = 0.11042; // 110.42ms
  public static final double VORTEX_FREE_SPEED = 6784; // RPM
  public static final double VORTEX_STALL_TORQUE = 3.6; // N * m

    public static final double GRAVITY = 9.81; // m/s/s
    public static final double FIELD_WIDTH = 8.0137;
    public static final double FIELD_LENGTH = 16.5410515;

  public static final double LOOP_CYCLE = 0.02; // 20ms

  public static final double ROBOT_MASS = 90;
  public static final double MANIPULATOR_MASS = 0;
  public static final double CHASSIS_MASS = ROBOT_MASS - MANIPULATOR_MASS;
  public static final double ARM_Y_POS = 0;
  public static final Translation3d CHASSIS_CG = new Translation3d(
    0,
    0,
    0.15);
    public static final class Drivebase {
        public static final int DEBUG_FLAG = 0b1;
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final int SWERVE_MODULE_CURRENT_LIMIT = 60;

        public static final double HEADING_TOLERANCE = Math.toRadians(1.5);

        // Motor and encoder inversions
        public static final boolean ABSOLUTE_ENCODER_INVERT = true;
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;
        public static final boolean INVERT_GYRO = false;

        // Module locations, in meters, as distances to the center of the robot.
        // Positive x is torwards the robot front, and +y is torwards robot left.
        public static final double FRONT_LEFT_X = Units.inchesToMeters(10.25);
        public static final double FRONT_LEFT_Y = Units.inchesToMeters(10.25);
        public static final double FRONT_RIGHT_X = Units.inchesToMeters(10.25);
        public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-10.25);
        public static final double BACK_LEFT_X = Units.inchesToMeters(-10.25);
        public static final double BACK_LEFT_Y = Units.inchesToMeters(10.25);
        public static final double BACK_RIGHT_X = Units.inchesToMeters(-10.25);
        public static final double BACK_RIGHT_Y = Units.inchesToMeters(-10.25);

        public static final Translation2d[] MODULE_LOCATIONS = {
                new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
                new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
                new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
                new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
        };

        // IMU Mounting. CCW Positive
        public static final double IMU_MOUNT_YAW = 0;
        public static final double IMU_MOUNT_PITCH = 0;
        public static final double IMU_MOUNT_ROLL = 0;

        // Drivetrain limitations
        public static final double MAX_SPEED = (VORTEX_FREE_SPEED * Units.inchesToMeters(3 * Math.PI)) / (60 * 4.71); // meters per second NOT A LIMIT!!! DO NOT TOUCH!!!
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y); // rad/s
        // Theoretical max acceleration should be as follows:
        // (NEO stall torque * module gearing * number of modules) / (wheel radius *
        // robot mass) = m/s/s
        // (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS)
        // However, the drive is traction-limited, so the max accelration is actually
        // (wheel coefficient of friction * gravity)
        public static final double MAX_ACCELERATION = 1 * GRAVITY; // COF is 1.1 but careful
        public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y);
        // max speed (RPM) / gear ratio, convert to deg/min, divide by 60 for deg/s
        public static final double MAX_MODULE_ANGULAR_SPEED = Units.rotationsToDegrees(NEO_550_FREE_SPEED * 7 / 372)
                / 60; // deg/sk

    // Currently does nothing
    public static final double ANGULAR_ACCELERATION_LIMIT = 100;
    public static final double ANGULAR_VELOCITY_LIMIT = 5;

        // Robot control gains
        public static final double HEADING_KP = 2;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;

    public static final double HEADING_MIN_ANGULAR_CONTROL_EFFORT = 0.05; // rad/s— Prevent oscillation by cancelling rotational commands less than this

        // Swerve base kinematics object
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    public static final double SKEW_CORRECTION_FACTOR = 0;

        // Module PIDF gains
        public static final double MODULE_KP = 0.06;
        public static final double MODULE_KI = 0.0;
        public static final double MODULE_KD = 0.0005;
        public static final double MODULE_IZ = 0;
        public static final double MODULE_KF = 0;
        // Volt * seconds / degree. Equal to (maxVolts) / ((degreesPerRotation) *
        // (maxMotorSpeedRPM / gearRatio) * (minutesPerSecond))
        public static final double MODULE_KV = 12 / MAX_MODULE_ANGULAR_SPEED;

        public static final double VELOCITY_KP = 0.06;
        public static final double VELOCITY_KI = 0.001; // Leave all of these zero to disable them
        public static final double VELOCITY_KD = 0;
        public static final double VELOCITY_IZ = 0;
        public static final double VELOCITY_KF = 0;

        public static final double CURRENT_KP = 0;
        public static final double CURRENT_KI = 0.2;
        public static final double CURRENT_KD = 0;
        public static final double CURRENT_MOVING_AVERAGE_SAMPLES = 1;

        public static final int NUM_MODULES = 4;

        // Drive feedforward gains
        public static final double KS = 0.12392; // Volts
        public static final double KV = 2.0891; // Volt-seconds per meter (max voltage divided by max speed)
        public static final double KA = 0.26159; // Volt-seconds^2 per meter (max voltage
                                                                               // divided by max accel)
        public static final double KG = (KA / KV);

        // Encoder conversion values. Drive converts motor rotations to linear wheel
        // distance
        // and steering converts motor rotations to module azimuth
        public static final double METERS_PER_MOTOR_ROTATION = (Math.PI * Units.inchesToMeters(3)) / 4.71;
        // Calculation: 3in diameter wheels * pi [circumfrence] / gear ratio
        public static final double DEGREES_PER_STEERING_ROTATION = 360;
        // degrees per rotation / gear ratio between module and motor

        // Module specific constants
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final double ANGLE_OFFSET = 180+360 - 213.5;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(0, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, FRONT_LEFT_X, FRONT_LEFT_Y);
        }

        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final double ANGLE_OFFSET = 180 - 53.7;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(1, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, FRONT_RIGHT_X, FRONT_RIGHT_Y);
        }

        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final double ANGLE_OFFSET = 180 - 168.6;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(2, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, BACK_LEFT_X, BACK_LEFT_Y);
        }

        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final double ANGLE_OFFSET = 360 - 310.3 + 180;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(3, DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, ANGLE_OFFSET, BACK_RIGHT_X, BACK_RIGHT_Y);
        }

        public static final int PIGEON = 30;
    }
    public static final class L1ArmConstants{

        public static final int SHOULDER_ID = 12;

        public static final int SMARTCURRENTLIMIT = 50;


        public static final boolean PRIMARY_ENCODER_POSITION_ALWAYS_ON = true;
        public static final int PRIMARY_ENCODER_POSITIONS_PERIOD = 20;
        public static final boolean PRIMARY_ENCODER_VELOCITY_ALWAYS_ON = true;
        public static final int PRIMARY_ENCODER_VELOCITY_PERIOD = 20;
        public static final boolean PRIMARY_ENCODER_INVERTED = false;
        public static final double PRIMARY_ENCODER_OFFSET = 0.7;
        public static final double SHOULDER_RADIANS_PER_PRIMARY_ENCODER_ROTATION = 2 * Math.PI/25 * 14/45;

        public static final boolean ABSOLUTE_ENCODER_POSITION_ALWAYS_ON = true;
        public static final int ABSOLUTE_ENCODER_POSITIONS_PERIOD = 20;
        public static final boolean ABSOLUTE_ENCODER_VELOCITY_ALWAYS_ON = true;
        public static final int ABSOLUTE_ENCODER_VELOCITY_PERIOD = 20;
        public static final boolean ABSOLUTE_ENCODER_INVERTED = true;
        public static final double ABSOLUTE_ENCODER_OFFSET = 146;
        public static final boolean ABSOLUTE_ENCODER_ZERO_CENTERED = true;
        public static final double SHOULDER_RADIANS_PER_ABS_ENCODER_ROTATION = 2 * Math.PI;

        public static final double KP = 0.8;
        public static final double KI = 0.0;
        public static final double KD = 0;
        public static final double KF = 0.0;

        public static final double KS = 0.113;
        public static final double KG = 0.162;
        public static final double KV = 1.65;
        public static final double KA = 0;
      
        
        public static final double OutputRangeMin = -1;
        public static final double OutputRangeMax = 1;

        public static final double MAX_SPEED = 1.788;
        public static final double MAX_ACCELERATION = 1.788;

        public static final Rotation2d STOWED = Rotation2d.fromDegrees(90);
        public static final Rotation2d UPPER_LIMIT = Rotation2d.fromDegrees(167);
        public static final Rotation2d LOWER_LIMIT = Rotation2d.fromDegrees(-39);
        public static final Rotation2d INTAKING = Rotation2d.fromDegrees(-38);
        public static final Rotation2d HUMANLOADING = Rotation2d.fromDegrees(65);
        public static final Rotation2d SCORING = Rotation2d.fromDegrees(160);
        public static final Rotation2d HAND_OFF = Rotation2d.fromDegrees(110);


        public static final double DEADBAND = Math.toRadians(0.1);

        public static final double TOLERANCE = Math.toRadians(0.1);
        public static final int SETTLE_TIME_LOOP_CYCLES = 200;

        public static final double CURRENT_OFFSET = 5;


    }
    public static final class L1IntakeConstants{
        public static final int INTAKE_ID = 16;

        public static final boolean INVERTED = true;

        public static final int SMARTCURRENTLIMIT = 30;
        public static final double OPEN_LOOP_RAMP_RATE = 10;

        public static final int FAULTS_PERIOD_MS = 20;

        public static final int PRIMARY_ENCODER_POSITON_PERIODMS = 6000;
        public static final int PRIMARY_ENCODER_VELOCITY_PERIODMS = 5900;

        public static final boolean OUTPUT_CURRENT_ALWAYS_ON = true;
        public static final int OUTPUT_CURRENT_PERIODMS = 20;

        public static final double MAX_SPEED = 1;

        public static final double INTAKING_CURRENT_THRESHOULD = 15;
        public static final double RELEASED_CURRENT_THRESHOULD = 10;
        public static final double NOPICKUP_CURRENT_THRESHOULD = 3;
        public static final int CYCLE_INTAKING_THRESHOLD = 50;
        public static final int CYCLE_RELEASED_THRESHOLD = 50;

        public static final double HOLDING_SPEED = 0.05;
        public static final double SCORE_SPEED = -1;
        public static final double INTAKE_SPEED = 0.35;

    }

    public static final class L4ArmConstants {
        //Values currently copied from L1ArmConstants, not updated 1/31/25
        public static final int SHOULDER_ID = 0;

        public static final int SMARTCURRENTLIMIT = 80;


        public static final boolean PRIMARY_ENCODER_POSITION_ALWAYS_ON = true;
        public static final int PRIMARY_ENCODER_POSITIONS_PERIOD = 20;
        public static final boolean PRIMARY_ENCODER_VELOCITY_ALWAYS_ON = true;
        public static final int PRIMARY_ENCODER_VELOCITY_PERIOD = 20;
        public static final boolean PRIMARY_ENCODER_INVERTED = true;
        public static final double PRIMARY_ENCODER_OFFSET = 0;
        public static final double SHOULDER_RADIANS_PER_PRIMARY_ENCODER_ROTATION = 2 * Math.PI * 16/48;

        public static final boolean ABSOLUTE_ENCODER_POSITION_ALWAYS_ON = true;
        public static final int ABSOLUTE_ENCODER_POSITIONS_PERIOD = 20;
        public static final boolean ABSOLUTE_ENCODER_VELOCITY_ALWAYS_ON = true;
        public static final int ABSOLUTE_ENCODER_VELOCITY_PERIOD = 20;
        public static final boolean ABSOLUTE_ENCODER_INVERTED = false;
        public static final double ABSOLUTE_ENCODER_OFFSET = 202;
        public static final double SHOULDER_RADIANS_PER_ABS_ENCODER_ROTATION = 2 * Math.PI;

        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0;

        public static final double KS = 0.0;
        public static final double KG = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
      
        
        public static final double OutputRangeMin = -1;
        public static final double OutputRangeMax = 1;

        public static final double MAX_SPEED = 10.0;
        public static final double MAX_ACCELERATION = 10.0;

        public static final Rotation2d STOWED = Rotation2d.fromDegrees(-50);
        public static final Rotation2d UPPER_LIMIT = Rotation2d.fromDegrees(135);
        public static final Rotation2d LOWER_LIMIT = Rotation2d.fromDegrees(-55);
        public static final Rotation2d INTAKING = Rotation2d.fromDegrees(-35);
        public static final Rotation2d SCORING = Rotation2d.fromDegrees(110);
        public static final Rotation2d HAND_OFF = Rotation2d.fromDegrees(-35);


        public static final double DEADBAND = Math.toRadians(0.1);

        public static final double TOLERANCE = Math.toRadians(0.1);
        public static final int SETTLE_TIME_LOOP_CYCLES = 10;

        public static final double CURRENT_OFFSET = 5;

    }
    public static final class Vision {
        public static final int DEBUG_FLAG = 0b10;

        public static final int APRILTAG_PIPELINE_NUMBER = 0;
        public static final String BOW_LIMELIGHT_NAME = "limelight-bow";
        public static final String STERN_LIMELIGHT_NAME = "limelight-stern";
        public static final String NOTE_LIMELIGHT_NAME = "limelight-note";

        @SuppressWarnings("unused")
        private static final int BOW_IP = 16; // Git-tracked notepad
        @SuppressWarnings("unused")
        private static final int STERN_IP = 15;
        @SuppressWarnings("unused")
        private static final int NOTE_IP = 14;

        public static final double POSE_ERROR_TOLERANCE = 1;
        public static final double ANGULAR_ERROR_TOLERANCE = Math.toRadians(5);
        public static final int LOOP_CYCLES_BEFORE_RESET = 20;

        public static final double ODOMETRY_TRANSLATIONAL_STD_DEV = 0.01; // Meters and radians
        public static final double ODOMETRY_ANGULAR_STD_DEV = 0.0002;

        public static final double XY_STD_DEV_COEFFICIENT = 0.01;
        public static final double ANG_STD_DEV_COEFFICIENT = 0.01;

        public static final double MAX_ALLOWABLE_Z_ERROR = 0.25; // Meters
    }

    public static final class Auton {
        // Plumbing via SmartDashboard
        public static final String ARM_ANGLE_KEY = "AutoArmAngleRadians";
        public static final String ON_TARGET_KEY = "AutoOnTarget";
        public static final String AUTO_SHOOTING_KEY = "AutoShootingRequested";
        public static final String PORT_SPEED_KEY = "PortSpeed";
        public static final String STARBOARD_SPEED_KEY = "StarboardSpeed";
        public static final String AUTO_AMP_ALIGN_KEY = "AutoAligningAmp";
        public static final String AUTO_AMP_SCORE_KEY = "AutoScoreAmp";
        public static final String AUTO_INTAKE_SPEED_KEY = "AutoIntakeSpeed";
        public static final String EJECT_MODE_KEY = "PurgeModeRequested";

        // Trapezoidal drive PID constants
        public static final double DRIVE_ACCELERATION_LIMIT = 2.1; // m/s/s
        public static final double DRIVE_VELOCITY_LIMIT = 6; // m/s
        public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            DRIVE_VELOCITY_LIMIT, DRIVE_ACCELERATION_LIMIT);
        
        public static final double ANGULAR_ACCELERATION_LIMIT = 1;
        public static final double ANGULAR_VELOCITY_LIMIT = 3;
        public static final TrapezoidProfile.Constraints ANGULAR_CONSTRAINTS = new TrapezoidProfile.Constraints(
            ANGULAR_VELOCITY_LIMIT, ANGULAR_ACCELERATION_LIMIT);
        
        public static final double DRIVE_KP = 2;
        public static final double DRIVE_KI = 0;
        public static final double DRIVE_KD = 0;

        public static final double DRIVE_POSITIONAL_TOLERANCE = 0.05; // m

        private static final Map<String, Pose2d> BLUE_MAP = Map.ofEntries(
           
        );
        // Iterates through every element in the pose map and mirrors them for the red alliance
        private static final Map<String, Pose2d> RED_MAP =
            BLUE_MAP.entrySet().stream().collect(Collectors.toMap(
                entry -> entry.getKey(),
                entry -> new Pose2d(
                    new Translation2d(
                        FIELD_LENGTH - entry.getValue().getX(),
                        entry.getValue().getY()
                    ),
                    new Rotation2d(
                        -entry.getValue().getRotation().getCos(),
                        entry.getValue().getRotation().getSin()
                    )
                )
            ));
        
        public static final Map<Alliance, Map<String, Pose2d>> POSE_MAP = Map.of(
            Alliance.Blue, BLUE_MAP,
            Alliance.Red, RED_MAP
        );

        // Added to final autoshoot calculations-- remember that we shoot from the back
        public static final Rotation2d AUTO_SHOOT_AZIMUTH_ADJUSTMENT = Rotation2d.fromDegrees(4);

        public static final double AUTO_INTAKE_SPEED = 1;
    }
    public static final class CommandDebugFlags {
        public static final int ALIGN_TO_POSE = 0b1000000;
        public static final int AUTO_SHOOT =    0b10000000;
        public static final int NOTE_HANDLER =  0b100000000;
        public static final int AUTO_AMP =      0b1000000000;
        public static final int ABS_DRIVE =     0b10000000000;
        public static final String FLAGS_KEY = "Debug Flags";
    }
    public static final class OperatorConstants {
        public static final int driveControllerPort = 0;
        public static final int headingControllerPort = 1;
        public static final int operatorControllerPort = 2;
    
    public static final double joystickDeadband = 0.05;
    }
}