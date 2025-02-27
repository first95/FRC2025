package frc.robot;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Drivebase;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final SparkMax angleMotor;
    private final SparkMaxConfig angleMotorConfig;
    private final SparkFlex driveMotor;
    private final SparkFlexConfig driveMotorConfig;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder driveEncoder;
    private final SparkClosedLoopController angleController, driveController;
    private final Timer time;
    private final Translation2d moduleLocation;
    private double angle, lastAngle, speed, fakePos, lastTime, dt;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private SimpleMotorFeedforward feedforward;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        angle = 0;
        speed = 0;
        fakePos = 0;
        moduleNumber = moduleConstants.moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        moduleLocation = new Translation2d(moduleConstants.xPos, moduleConstants.yPos);

        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleMotorConfig = new SparkMaxConfig();
        absoluteEncoder = angleMotor.getAbsoluteEncoder();
        angleController = angleMotor.getClosedLoopController();

        //config angle motor and controller
        angleMotorConfig
            .inverted(Drivebase.ANGLE_MOTOR_INVERT)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drivebase.SWERVE_MODULE_CURRENT_LIMIT);
        angleMotorConfig.absoluteEncoder
            .positionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION)
            .velocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION / 60)
            .zeroOffset(angleOffset/Drivebase.DEGREES_PER_STEERING_ROTATION)
            .inverted(Drivebase.ABSOLUTE_ENCODER_INVERT)
            .averageDepth(1);
        angleMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(Drivebase.MODULE_KP,
                  Drivebase.MODULE_KI,
                  Drivebase.MODULE_KD,
                  Drivebase.MODULE_KF)
            .iZone(Drivebase.MODULE_IZ)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180,180);
            
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            
            
        // Config drive motor/controller
        driveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveMotorConfig = new SparkFlexConfig();
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getClosedLoopController();
            
        driveMotorConfig
            .inverted(Drivebase.DRIVE_MOTOR_INVERT)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drivebase.SWERVE_MODULE_CURRENT_LIMIT);
        driveMotorConfig.encoder
            .positionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION)
            .velocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION / 60)
            .uvwAverageDepth(1); //uvwAverageDepth instead of quadratureAverageDepth, guessed based on old value
        driveMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(Drivebase.VELOCITY_KP,
                  Drivebase.VELOCITY_KI,
                  Drivebase.VELOCITY_KD,
                  Drivebase.VELOCITY_KF)
            .iZone(Drivebase.VELOCITY_IZ)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180,180);
            
            
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feedforward = new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA);

        time = new Timer();
        time.start();
        lastTime = time.get();
        
        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, true);
    }

    public void setVelocityGains(double kp, double ki, double kd, double ks, double kv, double ka) {
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        driveMotorConfig.closedLoop.pidf(kp,ki,kd,Drivebase.VELOCITY_KF);
        driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    public void setAzimuthGains(double kp, double ki, double kd) {
        angleMotorConfig.closedLoop.pid(kp,ki,kd);
        angleMotor.configure(angleMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean antijitter) {
        this.desiredState = desiredState;
        desiredState.optimize(getState().angle);

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Drivebase.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(velocity));
        }

        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (Drivebase.MAX_SPEED * 0.001)) && antijitter ? 
            lastAngle :
            desiredState.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
        angleController.setReference(angle, ControlType.kPosition);
        lastAngle = angle;

        this.angle = desiredState.angle.getDegrees();
        speed = desiredState.speedMetersPerSecond;

        if (!Robot.isReal()) {
            dt = time.get() - lastTime;
            fakePos += (speed * dt);
            lastTime = time.get();
        }
    }

    public void setAzimuth(Rotation2d azimuth) {
        angleController.setReference(azimuth.getDegrees(), ControlType.kPosition);
    }

    public void setDriveVolts(double volts) {
        driveMotor.setVoltage(volts);
    }

    public double getDriveVolts() {
        return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    }

    public double getDriveCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public Translation2d getModuleLocation() {
        return moduleLocation;
    }

    public SwerveModuleState getState() {
        double velocity;
        Rotation2d azimuth;
        if (Robot.isReal()) {
            velocity = driveEncoder.getVelocity();
            azimuth = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        } else {
            velocity = speed;
            azimuth = Rotation2d.fromDegrees(this.angle);
        }
        return new SwerveModuleState(velocity, azimuth);
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public SwerveModulePosition getPosition() {
        double position;
        Rotation2d azimuth;
        if (Robot.isReal()) {
            position = driveEncoder.getPosition();
            azimuth = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        } else {
            position = fakePos;
            azimuth = Rotation2d.fromDegrees(angle);
        }
        SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
        return new SwerveModulePosition(position, azimuth);
    }

    public double getAbsoluteEncoder() {
        return absoluteEncoder.getPosition();
    }

    public void setMotorBrake(boolean brake) {
        driveMotorConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        driveMotor.configure( driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void turnModule(double speed) {
        angleMotor.set(speed);
    }
}
