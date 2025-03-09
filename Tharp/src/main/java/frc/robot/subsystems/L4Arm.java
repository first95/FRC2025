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
import frc.robot.Constants.L4ArmConstants;

public class L4Arm extends SubsystemBase {
  private final SparkMax shoulder;
  private final SparkMaxConfig shoulderConfig;
  private final SparkClosedLoopController shoulderPID;
  private final SparkAbsoluteEncoder shoulderAbsoluteEncoder;
  private final SysIdRoutine L4Characterizer;

  //private final TrapezoidProfile shoulderProfile;
  private Rotation2d armGoal; 
  private final Timer timer;
  // private TrapezoidProfile.State profileStart, shoulderSetpoint; 
  private double lastShoulderVelocitySetpoint, armAccel;
  private int cyclesSinceShoulderNotAtGoal; 

  private ArmFeedforward shoulderFeedforward;
  private double shoulderFeedforwardValue;

  public L4Arm() {
    shoulder = new SparkMax(L4ArmConstants.SHOULDER_ID, MotorType.kBrushless);
    shoulderConfig = new SparkMaxConfig();
    shoulderPID = shoulder.getClosedLoopController();

    shoulderConfig
      .inverted(L4ArmConstants.PRIMARY_ENCODER_INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(L4ArmConstants.SMARTCURRENTLIMIT)
      .closedLoopRampRate(L4ArmConstants.CLOSEDLOOPRAMPRATE);
    
    shoulderConfig.signals
      .primaryEncoderPositionAlwaysOn(L4ArmConstants.PRIMARY_ENCODER_POSITION_ALWAYS_ON)
      .primaryEncoderPositionPeriodMs(L4ArmConstants.PRIMARY_ENCODER_POSITIONS_PERIOD)
      .primaryEncoderVelocityAlwaysOn(L4ArmConstants.PRIMARY_ENCODER_VELOCITY_ALWAYS_ON)
      .primaryEncoderVelocityPeriodMs(L4ArmConstants.PRIMARY_ENCODER_VELOCITY_PERIOD)
      .absoluteEncoderPositionAlwaysOn(L4ArmConstants.ABSOLUTE_ENCODER_POSITION_ALWAYS_ON)
      .absoluteEncoderPositionPeriodMs(L4ArmConstants.ABSOLUTE_ENCODER_POSITIONS_PERIOD)
      .absoluteEncoderVelocityAlwaysOn(L4ArmConstants.ABSOLUTE_ENCODER_VELOCITY_ALWAYS_ON)
      .absoluteEncoderVelocityPeriodMs(L4ArmConstants.ABSOLUTE_ENCODER_VELOCITY_PERIOD);

    shoulderConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pidf(L4ArmConstants.KP,
            L4ArmConstants.KI,
            L4ArmConstants.KD,
            L4ArmConstants.KF)
      .outputRange(L4ArmConstants.OutputRangeMin, L4ArmConstants.OutputRangeMax);
    
    
    shoulderConfig.absoluteEncoder
      .positionConversionFactor(L4ArmConstants.SHOULDER_RADIANS_PER_ABS_ENCODER_ROTATION)
      .velocityConversionFactor(L4ArmConstants.SHOULDER_RADIANS_PER_ABS_ENCODER_ROTATION)
      .zeroOffset(L4ArmConstants.ABSOLUTE_ENCODER_OFFSET/360)
      .inverted(L4ArmConstants.ABSOLUTE_ENCODER_INVERTED)
      .zeroCentered(true);  

    shoulderConfig.encoder
      .positionConversionFactor(L4ArmConstants.SHOULDER_RADIANS_PER_PRIMARY_ENCODER_ROTATION)
      .velocityConversionFactor(L4ArmConstants.SHOULDER_RADIANS_PER_PRIMARY_ENCODER_ROTATION/60);


    shoulderAbsoluteEncoder = shoulder.getAbsoluteEncoder();
    // shoulderProfile = new TrapezoidProfile(
    //   new TrapezoidProfile.Constraints(
    //     L4ArmConstants.MAX_SPEED, 
    //     L4ArmConstants.MAX_ACCELERATION));
    armGoal = L4ArmConstants.STOWED;
    // profileStart = new TrapezoidProfile.State(shoulderAbsoluteEncoder.getPosition(),0);
    // lastShoulderVelocitySetpoint = 0;
    armAccel = 0;

    shoulderFeedforward = new ArmFeedforward(
      L4ArmConstants.KS,
      L4ArmConstants.KG,
      L4ArmConstants.KV,
      L4ArmConstants.KA);

    shoulderFeedforwardValue = 0;

    shoulder.configure(shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    timer = new Timer();
    timer.start();

    cyclesSinceShoulderNotAtGoal = 0;
    L4Characterizer = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volatage -> {
              shoulder.setVoltage(volatage);
            },
            log -> {
              log.motor("L4Shoulder")
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  // public boolean armAtGoal() {
  //   return cyclesSinceShoulderNotAtGoal >= L4ArmConstants.SETTLE_TIME_LOOP_CYCLES;
  // }
  public boolean atGoal(){
    //return true;
    return Math.abs(shoulderAbsoluteEncoder.getPosition() - armGoal.getRadians()) < L4ArmConstants.TOLERANCE;
  }
  public boolean isMoving(){
    return shoulderAbsoluteEncoder.getVelocity() > L4ArmConstants.MOVING_THRESHOLD;
  }
  public Rotation2d getArmAngle(){
    return Rotation2d.fromRadians(shoulderAbsoluteEncoder.getPosition());
  }

  public double getArmCurrent(){
    return shoulder.getOutputCurrent();
  }

  public void setBrake(boolean setbrake){
    shoulderConfig.idleMode(setbrake ? IdleMode.kBrake : IdleMode.kCoast);
    shoulder.configure(shoulderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // public void setArmAngle(Rotation2d angle){
  //   if(angle.getRadians() != armGoal.getRadians()){
  //     armGoal = angle;
  //     profileStart = new TrapezoidProfile.State(shoulderAbsoluteEncoder.getPosition(),shoulderAbsoluteEncoder.getVelocity());
  //     cyclesSinceShoulderNotAtGoal = 0;
  //     timer.stop();
  //     timer.reset();
  //     timer.start();
  //   }
  // }
  public void setArmAngle(Rotation2d angle){
    //armGoal = Rotation2d.fromDegrees(SmartDashboard.getNumber("setShoulderAngleNumber", 0));
    armGoal = angle;
    if(armGoal.getRadians() >= L4ArmConstants.LOWER_LIMIT.getRadians() && armGoal.getRadians() <= L4ArmConstants.UPPER_LIMIT.getRadians()){
      shoulderPID.setReference(
      armGoal.getRadians(),
      ControlType.kPosition,
      ClosedLoopSlot.kSlot0,
      shoulderFeedforward.calculate(shoulderAbsoluteEncoder.getPosition(),0),
      ArbFFUnits.kVoltage);
    } 
  }
  public void setGains(){
    shoulderFeedforward = new ArmFeedforward(
    SmartDashboard.getNumber("shoulderKS",0), 
    SmartDashboard.getNumber("shoulderKG",0), 
    SmartDashboard.getNumber("shoulderKV", 0), 
    SmartDashboard.getNumber("shoulderKA", 0));
    shoulderConfig.closedLoop.pid(
    SmartDashboard.getNumber("shoulderKP",0), 
    SmartDashboard.getNumber("shoulderKI",0), 
    SmartDashboard.getNumber("shoulderKD", 0));
    shoulder.configure(shoulderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  @Override
  public void periodic() {
    
    if (armGoal == L4ArmConstants.STOWED && atGoal()) {
      shoulder.set(0);
    }

    // if(armGoal.getRadians() >= L4ArmConstants.UPPER_LIMIT.getRadians()){
    //   armGoal = L4ArmConstants.UPPER_LIMIT;
    // }
    // shoulderSetpoint = shoulderProfile.calculate(
    //   timer.get(), 
    //   new TrapezoidProfile.State(shoulderAbsoluteEncoder.getPosition(),shoulderAbsoluteEncoder.getVelocity()),
    //   new TrapezoidProfile.State(armGoal.getRadians(),0));

    //   if (shoulderSetpoint.velocity > lastShoulderVelocitySetpoint) {
    //     armAccel = L4ArmConstants.MAX_ACCELERATION;
    //   } else if (shoulderSetpoint.velocity < lastShoulderVelocitySetpoint) {
    //     armAccel = -L4ArmConstants.MAX_ACCELERATION;
    //   } else {
    //     armAccel = 0;
    //   }
    // lastShoulderVelocitySetpoint = shoulderSetpoint.velocity;

    // if (!armAtGoal() && Math.abs(shoulderSetpoint.position - L4ArmConstants.LOWER_LIMIT.getRadians()) <= L4ArmConstants.DEADBAND) {
    //   shoulder.set(0);
    //   shoulderFeedforwardValue = 0;
    // } else {
    //   shoulderFeedforwardValue = shoulderFeedforward.calculate(shoulderAbsoluteEncoder.getPosition(), shoulderSetpoint.velocity, armAccel);
    //   shoulderPID.setReference(
    //       shoulderSetpoint.position,
    //       ControlType.kPosition,
    //       ClosedLoopSlot.kSlot0,
    //       shoulderFeedforwardValue,
    //       ArbFFUnits.kVoltage);
    // }
    // cyclesSinceShoulderNotAtGoal = Math.abs(armGoal.getRadians() - shoulderAbsoluteEncoder.getPosition()) <= L4ArmConstants.TOLERANCE ?
    //   cyclesSinceShoulderNotAtGoal + 1 :
    //   0;
    
    
    SmartDashboard.putNumber("L4ShoulderGoal", armGoal.getDegrees());
    //SmartDashboard.putNumber("L4ShoulderSetpoint", Math.toDegrees(shoulderSetpoint.position));
    // SmartDashboard.putNumber("L4ShoulderSetpointVel", Math.toDegrees(shoulderSetpoint.velocity));
    SmartDashboard.putNumber("L4ShoulderPos", getArmAngle().getDegrees());
    SmartDashboard.putNumber("L4ShoulderVelocity", shoulderAbsoluteEncoder.getVelocity());
    SmartDashboard.putNumber("L4ShoulderControlEffort", shoulder.getAppliedOutput() * shoulder.getBusVoltage());
    SmartDashboard.putBoolean("L4ShoulderAtGoal", atGoal());
    SmartDashboard.putNumber("L4CycleCounter", cyclesSinceShoulderNotAtGoal);
    SmartDashboard.putNumber("L4SetpointAccel", Math.toDegrees(armAccel));
    SmartDashboard.putNumber("L4ShoulderCurrentDraw", getArmCurrent());
    SmartDashboard.putNumber("L4PrimaryEncoderPos",shoulder.getEncoder().getPosition());
    SmartDashboard.putBoolean("L4IsMoving", isMoving());

  }

  public Command sysIdDynShoulder(SysIdRoutine.Direction direction) {
    return L4Characterizer.dynamic(direction);
  }
  public Command sysIdQuasiShoulder(SysIdRoutine.Direction direction) {
    return L4Characterizer.quasistatic(direction);
  }


}
