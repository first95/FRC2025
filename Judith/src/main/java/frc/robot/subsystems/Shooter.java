// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
//import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.NoteHandlerSpeeds;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Vision;

public class Shooter extends SubsystemBase {

  private final CANSparkFlex portShooter, starboardShooter, shoulder;
  private final CANSparkMax loader; // shoulder2;
  private final RelativeEncoder portShooterEncoder, starboardShooterEncoder, shoulderEncoder;
  //private final SparkAbsoluteEncoder shoulderEncoder;
  private final SparkPIDController portShooterPID, starboardShooterPID, shoulderPID;
  private final TrapezoidProfile shoulderProfile;
  private final DigitalInput bottomLimitSwitch;
  private final DigitalInput noteSensor;
  private final SimpleMotorFeedforward flywheelFeedforward;
  private final ArmFeedforward shoulderFeedforward;
  private double armFeedforwardValue, lastArmVelocitySetpoint, armAccel, portTargetRPM, starboardTargetRPM;

  private final SysIdRoutine shoulderCharacterizer, portShootCharacterizer, starboardShootCharacterizer;

  private Rotation2d armGoal;
  private final Timer timer;
  private TrapezoidProfile.State armSetpoint, profileStart;
  private int cyclesSinceArmNotAtGoal, debugFlags;

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    portShooter = new CANSparkFlex(ShooterConstants.PORT_SHOOTER_ID, MotorType.kBrushless);
    starboardShooter = new CANSparkFlex(ShooterConstants.STARBOARD_SHOOTER_ID, MotorType.kBrushless);
    loader = new CANSparkMax(ShooterConstants.LOADER_ID, MotorType.kBrushless);
    shoulder = new CANSparkFlex(ArmConstants.SHOULDER_ID, MotorType.kBrushless);

    portShooter.restoreFactoryDefaults();
    starboardShooter.restoreFactoryDefaults();
    loader.restoreFactoryDefaults();
    shoulder.restoreFactoryDefaults();

    portShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, ShooterConstants.SHOOTER_STATUS_FRAME_0_PERIOD);
    portShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ShooterConstants.SHOOTER_STATUS_FRAME_1_PERIOD);
    portShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ShooterConstants.SHOOTER_STATUS_FRAME_2_PERIOD);

    starboardShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus0, ShooterConstants.SHOOTER_STATUS_FRAME_0_PERIOD);
    starboardShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ShooterConstants.SHOOTER_STATUS_FRAME_1_PERIOD);
    starboardShooter.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ShooterConstants.SHOOTER_STATUS_FRAME_2_PERIOD);

    loader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, ShooterConstants.LOADER_STATUS_FRAME_0_PERIOD);
    loader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, ShooterConstants.LOADER_STATUS_FRAME_1_PERIOD);
    loader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, ShooterConstants.LOADER_STATUS_FRAME_2_PERIOD);

    shoulder.setPeriodicFramePeriod(PeriodicFrame.kStatus5, ArmConstants.ABS_ENCODER_STATUS_FRAME_PERIOD_MS);
    shoulder.setPeriodicFramePeriod(PeriodicFrame.kStatus6, ArmConstants.ABS_ENCODER_STATUS_FRAME_PERIOD_MS);

    portShooter.setInverted(ShooterConstants.INVERT_PORT_SHOOTER);
    starboardShooter.setInverted(ShooterConstants.INVERT_STARBOARD_SHOOTER);
    loader.setInverted(ShooterConstants.INVERT_LOADER);
    shoulder.setInverted(ArmConstants.INVERT_SHOULDER);

    portShooter.setIdleMode(IdleMode.kCoast);
    starboardShooter.setIdleMode(IdleMode.kCoast);
    loader.setIdleMode(IdleMode.kBrake);
    shoulder.setIdleMode(IdleMode.kBrake);

    portShooter.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    starboardShooter.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    loader.setSmartCurrentLimit(ShooterConstants.LOADER_CURRENT_LIMIT);
    shoulder.setSmartCurrentLimit(ArmConstants.SHOULDER_CURRENT_LIMIT);

    portShooter.setOpenLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    starboardShooter.setOpenLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    portShooter.setClosedLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    starboardShooter.setClosedLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);

    portShooterEncoder = portShooter.getEncoder();
    starboardShooterEncoder = starboardShooter.getEncoder();
    //shoulderEncoder = shoulder.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderEncoder = shoulder.getEncoder();

    shoulderEncoder.setPositionConversionFactor(ArmConstants.RADIANS_PER_ENCODER_ROTATION);
    shoulderEncoder.setVelocityConversionFactor(ArmConstants.RADIANS_PER_ENCODER_ROTATION / 60);
    //shoulderEncoder.setInverted(ArmConstants.INVERT_ENCODER);
    //shoulderEncoder.setZeroOffset(ArmConstants.ZERO_OFFSET.getRadians());
    //shoulderEncoder.setAverageDepth(1);

    portShooterPID = portShooter.getPIDController();
    starboardShooterPID = starboardShooter.getPIDController();
    shoulderPID = shoulder.getPIDController();

    portShooterPID.setP(ShooterConstants.KP);
    portShooterPID.setI(ShooterConstants.KI);
    portShooterPID.setD(ShooterConstants.KD);
    portShooterPID.setFF(ShooterConstants.KF);

    starboardShooterPID.setP(ShooterConstants.KP);
    starboardShooterPID.setI(ShooterConstants.KI);
    starboardShooterPID.setD(ShooterConstants.KD);
    starboardShooterPID.setFF(ShooterConstants.KF);

    portTargetRPM = NoteHandlerSpeeds.PORT_IDLE;
    starboardTargetRPM = NoteHandlerSpeeds.STARBOARD_IDLE;

    shoulderPID.setP(ArmConstants.KP);
    shoulderPID.setI(ArmConstants.KI);
    shoulderPID.setD(ArmConstants.KD);
    shoulderPID.setFF(ArmConstants.KF);

    shoulderPID.setFeedbackDevice(shoulderEncoder);

    shoulderPID.setOutputRange(ArmConstants.MIN_CONTROL_EFFORT,
        ArmConstants.MAX_CONTROL_EFFORT);

    bottomLimitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_ID);

    shoulderProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ArmConstants.MAX_SPEED,
            ArmConstants.MAX_ACCELERATION));
    armGoal = ArmConstants.LOWER_LIMIT;
    profileStart = new TrapezoidProfile.State(ArmConstants.LOWER_LIMIT.getRadians(), 0);
    lastArmVelocitySetpoint = 0;
    armAccel = 0;

    shoulderFeedforward = new ArmFeedforward(
        ArmConstants.KS,
        ArmConstants.KG,
        ArmConstants.KV,
        ArmConstants.KA);
    
    armFeedforwardValue = 0;

    flywheelFeedforward = new SimpleMotorFeedforward(
        ShooterConstants.KS,
        ShooterConstants.KV,
        ShooterConstants.KA);

    noteSensor = new DigitalInput(ShooterConstants.NOTE_SENSOR_ID);

    portShooter.burnFlash();
    starboardShooter.burnFlash();
    loader.burnFlash();
    shoulder.burnFlash();

    timer = new Timer();
    timer.start();

    cyclesSinceArmNotAtGoal = 0;

    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    shoulderCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              shoulder.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("shoulder")
                  .voltage(Volts.of(shoulder.getAppliedOutput() * shoulder.getBusVoltage()))
                  .angularPosition(Radians.of(shoulderEncoder.getPosition()))
                  .angularVelocity(RadiansPerSecond.of(shoulderEncoder.getVelocity()));
            },
            this));

    portShootCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              portShooter.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("portShooter")
                  .voltage(Volts.of(portShooter.getAppliedOutput() * portShooter.getBusVoltage()))
                  .angularPosition(Rotations.of(portShooterEncoder.getPosition()))
                  .angularVelocity(RotationsPerSecond.of(portShooterEncoder.getVelocity()));
            },
            this));

    starboardShootCharacterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
              starboardShooter.setVoltage(volts.in(Volts));
            },
            log -> {
              log.motor("starboardShooter")
                  .voltage(Volts.of(starboardShooter.getAppliedOutput() * starboardShooter.getBusVoltage()))
                  .angularPosition(Rotations.of(starboardShooterEncoder.getPosition()))
                  .angularVelocity(RotationsPerSecond.of(starboardShooterEncoder.getVelocity()));
            },
            this));
  }

  public void runLoader(double speed) {
    loader.set(speed);
  }

  public void setShooterSpeed(double portRPM, double starboardRPM) {
    portTargetRPM = portRPM;
    starboardTargetRPM = starboardRPM;
    portShooterPID.setReference(portRPM, ControlType.kVelocity, 0, flywheelFeedforward.calculate(portRPM),
        ArbFFUnits.kVoltage);
    starboardShooterPID.setReference(starboardRPM, ControlType.kVelocity, 0,
        flywheelFeedforward.calculate(starboardRPM), ArbFFUnits.kVoltage);
  }

  public boolean shootersAtSpeeds() {
    return Math.abs(portTargetRPM - portShooterEncoder.getVelocity()) <= NoteHandlerSpeeds.SHOOTER_TOLERANCE
      && Math.abs(starboardTargetRPM - starboardShooterEncoder.getVelocity()) <= NoteHandlerSpeeds.SHOOTER_TOLERANCE;
  }

  public void setPortShooterRaw(double speed) {
    portShooter.set(speed);
  }

  public void setStarboardShooterRaw(double speed) {
    starboardShooter.set(speed);
  }

  public void setArmAngle(Rotation2d angle) {
    if (angle.getRadians() != armGoal.getRadians()) {
      armGoal = angle;
      profileStart = new TrapezoidProfile.State(shoulderEncoder.getPosition(), shoulderEncoder.getVelocity());
      cyclesSinceArmNotAtGoal = 0;
      timer.stop();
      timer.reset();
      timer.start();
    }
  }

  public void setShoulderVolts(double volts) {
    shoulder.setVoltage(volts);
  }

  public Rotation2d getArmAngle() {
    return Rotation2d.fromRadians(shoulderEncoder.getPosition());
  }

  public double getPortShooterSpeed() {
    return portShooterEncoder.getVelocity();
  }

  public double getStarboardShooterSpeed() {
    return starboardShooterEncoder.getVelocity();
  }

  public boolean getNoteSensor() {
    return noteSensor.get();
  }

  public boolean armAtGoal() {
    return cyclesSinceArmNotAtGoal >= ArmConstants.SETTLE_TIME_LOOP_CYCLES;
  }

  public void zeroEncoder() {
    //shoulderEncoder.setZeroOffset((shoulderEncoder.getPosition() + shoulderEncoder.getZeroOffset()) - ArmConstants.LOWER_LIMIT.getRadians());
  }


  @Override
  public void periodic() {
    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    if (bottomLimitSwitch.get()) {
      shoulderEncoder.setPosition(ArmConstants.LOWER_LIMIT.getRadians());
    }

    if (armGoal.getRadians() >= ArmConstants.UPPER_LIMIT.getRadians()) {
      armGoal = ArmConstants.UPPER_LIMIT;
    }
    armSetpoint = shoulderProfile.calculate(
        timer.get(),
        profileStart,
        new TrapezoidProfile.State(armGoal.getRadians(), 0));
    // This really shouldn't be necessary
    armSetpoint = (armSetpoint.position >= ArmConstants.UPPER_LIMIT.getRadians())
        ? new TrapezoidProfile.State(ArmConstants.UPPER_LIMIT.getRadians(), 0)
        : armSetpoint;

    if (armSetpoint.velocity > lastArmVelocitySetpoint) {
      armAccel = ArmConstants.MAX_ACCELERATION;
    } else if (armSetpoint.velocity < lastArmVelocitySetpoint) {
      armAccel = -ArmConstants.MAX_ACCELERATION;
    } else {
      armAccel = 0;
    }

    lastArmVelocitySetpoint = armSetpoint.velocity;

    if (!armAtGoal() && (Math.abs(armSetpoint.position - ArmConstants.LOWER_LIMIT.getRadians()) <= ArmConstants.DEADBAND)) {
      shoulder.set(0);
      armFeedforwardValue = 0;
    } else {
      armFeedforwardValue = shoulderFeedforward.calculate(armSetpoint.position, armSetpoint.velocity, armAccel);
      shoulderPID.setReference(
          armSetpoint.position,
          ControlType.kPosition,
          0,
          armFeedforwardValue,
          ArbFFUnits.kVoltage);
    }

    if (noteSensor.get()) {
      LimelightHelpers.setLEDMode_ForceOn(Vision.NOTE_LIMELIGHT_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(Vision.NOTE_LIMELIGHT_NAME);
    }

    cyclesSinceArmNotAtGoal = Math.abs(armGoal.getRadians() - shoulderEncoder.getPosition()) <= ArmConstants.TOLERANCE ?
      cyclesSinceArmNotAtGoal + 1 :
      0;

    if ((debugFlags & ArmConstants.DEBUG_FLAG) != 0) {
      SmartDashboard.putNumber("ProportionalTerm",
      ArmConstants.KP * (armSetpoint.position - shoulderEncoder.getPosition()));
      SmartDashboard.putNumber("FeedforwardValue", armFeedforwardValue);
      SmartDashboard.putBoolean("LimitSwitch", bottomLimitSwitch.get());
      SmartDashboard.putNumber("ShooterShoulderGoal", armGoal.getDegrees());
      SmartDashboard.putNumber("ShooterShoulderSetpoint", Math.toDegrees(armSetpoint.position));
      SmartDashboard.putNumber("ShooterShoulderSetpointVel", Math.toDegrees(armSetpoint.velocity));
      SmartDashboard.putNumber("ShooterShoulderPos", getArmAngle().getDegrees());
      SmartDashboard.putNumber("ShoulderControlEffort", shoulder.getAppliedOutput() * shoulder.getBusVoltage());
      SmartDashboard.putBoolean("ShoulderAtGoal", armAtGoal());
      SmartDashboard.putNumber("CycleCounter", cyclesSinceArmNotAtGoal);
      SmartDashboard.putNumber("ShooterSetpointAccel", Math.toDegrees(armAccel));
    }
    if ((debugFlags & ShooterConstants.DEBUG_FLAG) != 0) {
      SmartDashboard.putNumber("PortVolts", portShooter.getAppliedOutput() * portShooter.getBusVoltage());
      SmartDashboard.putNumber("StarboardVolts", starboardShooter.getAppliedOutput() * starboardShooter.getBusVoltage());
      SmartDashboard.putNumber("PortRPM", portShooterEncoder.getVelocity());
      SmartDashboard.putNumber("StarboardRPM", starboardShooterEncoder.getVelocity());
    }
  }

  public Command sysIdQuasiShoulder(SysIdRoutine.Direction direction) {
    return shoulderCharacterizer.quasistatic(direction);
  }

  public Command sysIdQuasiPortShooter(SysIdRoutine.Direction direction) {
    return portShootCharacterizer.quasistatic(direction);
  }

  public Command sysIdQuasiStarShooter(SysIdRoutine.Direction direction) {
    return starboardShootCharacterizer.quasistatic(direction);
  }

  public Command sysIdDynShoulder(SysIdRoutine.Direction direction) {
    return shoulderCharacterizer.dynamic(direction);
  }

  public Command sysIdDynPortShooter(SysIdRoutine.Direction direction) {
    return portShootCharacterizer.dynamic(direction);
  }

  public Command sysIdDynStarShooter(SysIdRoutine.Direction direction) {
    return starboardShootCharacterizer.dynamic(direction);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
