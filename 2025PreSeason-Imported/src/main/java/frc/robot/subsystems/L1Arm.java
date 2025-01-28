// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.Constants.L1ArmConstants;
import frc.robot.Constants.L1IntakeConstants;


public class L1Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax intake;
  private final SparkMaxConfig intakeConfig;  

  private final SparkMax shoulder;
  private final SparkMaxConfig shoulderConfig;
  private final SparkClosedLoopController shoulderPID;
  private final SparkAbsoluteEncoder shoulderAbsoluteEncoder;
  private final RelativeEncoder shoulderRelativeEncoder;

  private final TrapezoidProfile shoulderProfile;
  private Rotation2d armGoal; 
  private final Timer timer;
  private TrapezoidProfile.State profileStart, shoulderSetpoint; 
  private double lastShoulderVelocitySetpoint, armAccel;
  private int cyclesSinceShoulderNotAtGoal; 

  private final ArmFeedforward shoulderFeedforward;
  private double shoulderFeedforwardValue;

  private final SysIdRoutine shoulderCharacterizer;


  public L1Arm() {
    // configure intake motor parameters
    intake = new SparkMax(L1IntakeConstants.INTAKE_ID, MotorType.kBrushless);
    intakeConfig = new SparkMaxConfig();

    intakeConfig
      .inverted(L1IntakeConstants.INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(L1IntakeConstants.SMARTCURRENTLIMIT);
    intakeConfig.signals
      .faultsPeriodMs(L1IntakeConstants.FAULTS_PERIOD_MS)
      .primaryEncoderPositionPeriodMs(L1IntakeConstants.PRIMARY_ENCODER_POSITON_PERIODMS)
      .primaryEncoderVelocityPeriodMs(L1IntakeConstants.PRIMARY_ENCODER_VELOCITY_PERIODMS)
      .outputCurrentPeriodMs(L1IntakeConstants.OUTPUT_CURRENT_PERIODMS);

    // write the intake motor parameters to the SparkMAX and write to flash  
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    shoulder = new SparkMax(L1ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    shoulderConfig = new SparkMaxConfig();
    shoulderPID = shoulder.getClosedLoopController();

    
    shoulderConfig
      .inverted(L1ArmConstants.PRIMARY_ENCODER_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(L1ArmConstants.SMARTCURRENTLIMIT);
    shoulderConfig.signals
      .primaryEncoderPositionAlwaysOn(L1ArmConstants.PRIMARY_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderPositionPeriodMs(L1ArmConstants.PRIMARY_ENCODER_POSITIONS_PERIOD)
      .primaryEncoderVelocityAlwaysOn(L1ArmConstants.PRIMARY_ENCODER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(L1ArmConstants.PRIMARY_ENCODER_VELOCITY_PERIOD)
      .absoluteEncoderPositionAlwaysOn(L1ArmConstants.ABSOLUTE_ENCODER_POSITION_ALWAYS_ON)
      .absoluteEncoderPositionPeriodMs(L1ArmConstants.ABSOLUTE_ENCODER_POSITIONS_PERIOD)
      .absoluteEncoderVelocityAlwaysOn(L1ArmConstants.ABSOLUTE_ENCODER_VELOCITY_ALWAYS_ON)
      .absoluteEncoderVelocityPeriodMs(L1ArmConstants.ABSOLUTE_ENCODER_VELOCITY_PERIOD);
    shoulderConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pidf(L1ArmConstants.KP,
            L1ArmConstants.KI,
            L1ArmConstants.KD,
            L1ArmConstants.KF)
      .outputRange(L1ArmConstants.OutputRangeMin, L1ArmConstants.OutputRangeMax);
    shoulderConfig.absoluteEncoder
      .positionConversionFactor(L1ArmConstants.SHOULDER_RADIANS_PER_ABS_ENCODER_ROTATION)
      .velocityConversionFactor(L1ArmConstants.SHOULDER_RADIANS_PER_ABS_ENCODER_ROTATION/ 60)
      .zeroOffset(L1ArmConstants.ABSOLUTE_ENCODER_OFFSET/360)
      .inverted(L1ArmConstants.ABSOLUTE_ENCODER_INVERTED)
      .zeroCentered(L1ArmConstants.ABSOLUTE_ENCODER_ZERO_CENTERED);     
    shoulderConfig.encoder
      .positionConversionFactor(L1ArmConstants.SHOULDER_RADIANS_PER_PRIMARY_ENCODER_ROTATION)
      .velocityConversionFactor(L1ArmConstants.SHOULDER_RADIANS_PER_PRIMARY_ENCODER_ROTATION/60);
    
    shoulderProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        L1ArmConstants.MAX_SPEED, 
        L1ArmConstants.MAX_ACCELERATION));
    // armGoal = L1ArmConstants.STOWED;
    // profileStart = new TrapezoidProfile.State(shoulder.getEncoder().getPosition(),0);
    lastShoulderVelocitySetpoint = 0;
    armAccel = 0;

    shoulderFeedforward = new ArmFeedforward(
      L1ArmConstants.KS,
      L1ArmConstants.KG,
      L1ArmConstants.KV,
      L1ArmConstants.KA);

    shoulderFeedforwardValue = 0;

    shoulder.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shoulderAbsoluteEncoder = shoulder.getAbsoluteEncoder();
    shoulderRelativeEncoder = shoulder.getEncoder();
    shoulderRelativeEncoder.setPosition(shoulderRelativeEncoder.getPosition());

    timer = new Timer();
    timer.start();

    cyclesSinceShoulderNotAtGoal = 0;

    shoulderCharacterizer = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        voltage ->{
          shoulder.setVoltage(voltage);
        },
      log -> {
        log.motor("L1Shoulder")
          .voltage(Volts.of(shoulder.getAppliedOutput() * shoulder.getBusVoltage()))
          .angularPosition(Radians.of(shoulderAbsoluteEncoder.getPosition()))
          .angularVelocity(RadiansPerSecond.of(shoulderAbsoluteEncoder.getVelocity()));
      }, 
      this));
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public Command sysIdQuasiShoulder(SysIdRoutine.Direction direction) {
    return shoulderCharacterizer.quasistatic(direction);
  }
  public Command sysIdDynShoulder(SysIdRoutine.Direction direction) {
    return shoulderCharacterizer.dynamic(direction);
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public double getArmVoltage(){
    return (shoulder.getAppliedOutput() * shoulder.getBusVoltage());
  }
  public void incrementArmVoltage(double increment){
    shoulder.set(shoulder.getAppliedOutput() + increment);
  }
  public boolean armAtGoal() {
    return cyclesSinceShoulderNotAtGoal >= L1ArmConstants.SETTLE_TIME_LOOP_CYCLES;
  }

  public Rotation2d getArmAngle(){
    return Rotation2d.fromRadians(shoulderAbsoluteEncoder.getPosition());
  }
  public Rotation2d getRelativeEncoderPos(){
    return Rotation2d.fromRadians(shoulderRelativeEncoder.getPosition());
  }
  public double getArmCurrent(){
    return shoulder.getOutputCurrent();
  }

  public void runIntake(double speed){
    intake.set(speed * L1IntakeConstants.MAX_SPEED);
  }
  
  public double getIntakeCurrent(){
    return intake.getOutputCurrent();
  }

  public void setArmAngle(Rotation2d angle){
    if(angle.getRadians() != armGoal.getRadians()){
      armGoal = angle;
      profileStart = new TrapezoidProfile.State(shoulderAbsoluteEncoder.getPosition(),shoulderAbsoluteEncoder.getVelocity());
      cyclesSinceShoulderNotAtGoal = 0;
      timer.stop();
      timer.reset();
      timer.start();
    }
  }
  @Override
  public void periodic() {
    shoulder.setVoltage(shoulderFeedforward.calculate(shoulderAbsoluteEncoder.getPosition(), 0));

  //   if(armGoal.getRadians() >= L1ArmConstants.UPPER_LIMIT.getRadians()){
  //     armGoal = L1ArmConstants.UPPER_LIMIT;
  //   }
  //   shoulderSetpoint = shoulderProfile.calculate(
  //     timer.get(), 
  //     profileStart,
  //     new TrapezoidProfile.State(armGoal.getRadians(),0));

  //     if (shoulderSetpoint.velocity > lastShoulderVelocitySetpoint) {
  //       armAccel = L1ArmConstants.MAX_ACCELERATION;
  //     } else if (shoulderSetpoint.velocity < lastShoulderVelocitySetpoint) {
  //       armAccel = -L1ArmConstants.MAX_ACCELERATION;
  //     } else {
  //       armAccel = 0;
  //     }
  //   lastShoulderVelocitySetpoint = shoulderSetpoint.velocity;

  //   if (!armAtGoal() && ((Math.abs(shoulderSetpoint.position - L1ArmConstants.LOWER_LIMIT.getRadians()) <= L1ArmConstants.DEADBAND) ||
  //                       (Math.abs(shoulderSetpoint.position - L1ArmConstants.UPPER_LIMIT.getRadians()) <= L1ArmConstants.DEADBAND))) {
  //     shoulder.set(0);
  //     shoulderFeedforwardValue = 0;
  //   } else {
  //     shoulderFeedforwardValue = shoulderFeedforward.calculate(shoulderSetpoint.position, shoulderSetpoint.velocity, armAccel);
  //     shoulderPID.setReference(
  //         shoulderSetpoint.position,
  //         ControlType.kPosition,
  //         ClosedLoopSlot.kSlot0,
  //         shoulderFeedforwardValue,
  //         ArbFFUnits.kVoltage);
  //   }
  //   cyclesSinceShoulderNotAtGoal = Math.abs(armGoal.getRadians() - shoulderAbsoluteEncoder.getPosition()) <= L1ArmConstants.TOLERANCE ?
  //     cyclesSinceShoulderNotAtGoal + 1 :
  //     0;
    
    
  //   SmartDashboard.putNumber("ShoulderGoal", armGoal.getDegrees());
  //   SmartDashboard.putNumber("ShoulderSetpoint", Math.toDegrees(shoulderSetpoint.position));
  //  SmartDashboard.putNumber("ShoulderSetpointVel", Math.toDegrees(shoulderSetpoint.velocity));
    SmartDashboard.putNumber("ShoulderPos", getArmAngle().getDegrees());
    SmartDashboard.putNumber("ShoulderControlEffort", shoulder.getAppliedOutput() * shoulder.getBusVoltage());
    SmartDashboard.putBoolean("ShoulderAtGoal", armAtGoal());
    SmartDashboard.putNumber("CycleCounter", cyclesSinceShoulderNotAtGoal);
    SmartDashboard.putNumber("SetpointAccel", Math.toDegrees(armAccel));
    SmartDashboard.putNumber("ShoulderCurrentDraw", getArmCurrent());
    SmartDashboard.putNumber("ArmVoltage", shoulder.getAppliedOutput() * shoulder.getBusVoltage());
    SmartDashboard.putNumber("PrimaryEncoderPos",getRelativeEncoderPos().getDegrees());

    SmartDashboard.putNumber("IntakeCurrentDraw",getIntakeCurrent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
