// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.NoteHandlerSpeeds;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final DoubleSupplier intakeSpeedAxis;
  private final BooleanSupplier shooterButtonSupplier, ampAlignSupplier, ampScoreSupplier,
    hpStationLoadSupplier, ejectSupplier, unjamSupplier, climbSupplier, feederSupplier, lowFeederSupplier;

  private enum State {
    SHOOTING, IDLE, HOLDING, SPOOLING, AUTO_SPOOLING, AUTO_SHOOTING, INDEXING_REV, INDEXING_FWD, AMP_ALIGNING,
    AMP_SCORING_A, AMP_SCORING_B, HP_STATION_LOAD, CLIMB, UNJAM, EJECT_DOWNSPOOL, EJECT, FEEDER_SHOOTING, FEEDER_SPOOLING,
    LOW_FEEDER_SPOOLING, LOW_FEEDER_SHOOTING;
  }

  private State currentState;
  private double commandedIntakeSpeed;
  private Rotation2d autoArmAngle;
  private boolean sensorvalue, shooterbutton, shooterAtSpeed, autoShooting, onTarget, armInPosition,
    ampAligning, ampScoring, hpLoading, ejectButton, unjamButton, climbButton, feederButton, lowFeederButton;
  private int debugFlags;

  public NoteHandlerCommand(
      Shooter shooter,
      Intake intake,
      DoubleSupplier intakeSpeedAxis,
      BooleanSupplier shooterButtonSupplier,
      BooleanSupplier ampAlignSupplier,
      BooleanSupplier ampScoreSupplier,
      BooleanSupplier playerStationLoadSupplier,
      BooleanSupplier ejectSupplier,
      BooleanSupplier unjamSupplier,
      BooleanSupplier climbSupplier,
      BooleanSupplier feederSupplier,
      BooleanSupplier lowFeederSupplier) {
    this.shooter = shooter;
    this.intake = intake;
    this.intakeSpeedAxis = intakeSpeedAxis;
    this.shooterButtonSupplier = shooterButtonSupplier;
    this.ampAlignSupplier = ampAlignSupplier;
    this.ampScoreSupplier = ampScoreSupplier;
    this.hpStationLoadSupplier = playerStationLoadSupplier;
    this.ejectSupplier = ejectSupplier;
    this.unjamSupplier = unjamSupplier;
    this.climbSupplier = climbSupplier;
    this.feederSupplier = feederSupplier;
    this.lowFeederSupplier = lowFeederSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;

    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_IDLE, NoteHandlerSpeeds.STARBOARD_IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read inputs
    sensorvalue = shooter.getNoteSensor();
    shooterbutton = shooterButtonSupplier.getAsBoolean();
    commandedIntakeSpeed = intakeSpeedAxis.getAsDouble() + SmartDashboard.getNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);
    autoShooting = SmartDashboard.getBoolean(Auton.AUTO_SHOOTING_KEY, false);
    onTarget = SmartDashboard.getBoolean(Auton.ON_TARGET_KEY, false);
    autoArmAngle = Rotation2d
        .fromRadians(SmartDashboard.getNumber(Auton.ARM_ANGLE_KEY, ArmConstants.LOWER_LIMIT.getRadians()));
    ampAligning = ampAlignSupplier.getAsBoolean() || SmartDashboard.getBoolean(Auton.AUTO_AMP_ALIGN_KEY, false);
    ampScoring = ampScoreSupplier.getAsBoolean() || SmartDashboard.getBoolean(Auton.AUTO_AMP_SCORE_KEY, false);
    hpLoading = hpStationLoadSupplier.getAsBoolean();
    ejectButton = ejectSupplier.getAsBoolean() || SmartDashboard.getBoolean(Auton.EJECT_MODE_KEY, false);
    unjamButton = unjamSupplier.getAsBoolean();
    climbButton = climbSupplier.getAsBoolean();
    feederButton = feederSupplier.getAsBoolean();
    lowFeederButton = lowFeederSupplier.getAsBoolean();

    // Execute statemachine logic
    switch (currentState) {
      case IDLE:
        // Set desired outputs in this state
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader((commandedIntakeSpeed > 0) ? NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_IDLE, NoteHandlerSpeeds.STARBOARD_IDLE);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        // Test output-dependent conditions
        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        // Determine if we neeed to change state
        if (sensorvalue) {
          currentState = State.INDEXING_REV;
        }
        if (hpLoading) {
          currentState = State.HP_STATION_LOAD;
        }
        if (climbButton) {
          currentState = State.CLIMB;
        }
        if (ampAligning) {
          currentState = State.AMP_ALIGNING;
        }
        if (ampAligning && ampScoring) {
          currentState = State.AMP_SCORING_A;
        }
        if (ejectButton) {
          currentState = State.EJECT_DOWNSPOOL;
        }
        if (unjamButton) {
          currentState = State.UNJAM;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }

        // Don't forget this-- things get real weird when every successive case can
        // execute
      break;

      case SPOOLING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_SHOOTER, NoteHandlerSpeeds.STARBOARD_SHOOTER);
        shooter.setArmAngle(ArmConstants.MANUAL_SHOT_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!shooterbutton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (shooterbutton && shooterAtSpeed && armInPosition) {
          currentState = State.SHOOTING;
        }

      break;

      case AUTO_SPOOLING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_SHOOTER, NoteHandlerSpeeds.STARBOARD_SHOOTER);
        shooter.setArmAngle(autoArmAngle);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!autoShooting) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (armInPosition && shooterAtSpeed && onTarget && autoShooting) {
          currentState = State.AUTO_SHOOTING;
        }

      break;
      
      case FEEDER_SPOOLING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_FEEDER, NoteHandlerSpeeds.STARBOARD_FEEDER);
        shooter.setArmAngle(ArmConstants.FEEDER_SHOT_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!feederButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (armInPosition && shooterAtSpeed && feederButton) {
          currentState = State.FEEDER_SHOOTING;
        }

      break;

      case LOW_FEEDER_SPOOLING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_LOW_FEEDER, NoteHandlerSpeeds.STARBOARD_LOW_FEEDER);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!lowFeederButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (armInPosition && shooterAtSpeed && lowFeederButton) {
          currentState = State.LOW_FEEDER_SHOOTING;
        }

      break;

      case SHOOTING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_SHOOTER, NoteHandlerSpeeds.STARBOARD_SHOOTER);
        shooter.setArmAngle(ArmConstants.MANUAL_SHOT_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!shooterbutton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }

      break;

      case AUTO_SHOOTING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_SHOOTER, NoteHandlerSpeeds.STARBOARD_SHOOTER);
        shooter.setArmAngle(autoArmAngle);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!autoShooting) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }

      break;
      
      case FEEDER_SHOOTING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_FEEDER, NoteHandlerSpeeds.STARBOARD_FEEDER);
        shooter.setArmAngle(ArmConstants.FEEDER_SHOT_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!feederButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }

      break;

      case LOW_FEEDER_SHOOTING:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_LOW_FEEDER, NoteHandlerSpeeds.STARBOARD_LOW_FEEDER);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!lowFeederButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }

      break;

      case INDEXING_REV:
        intake.runRollers(commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(-NoteHandlerSpeeds.LOADER_INDEXING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_IDLE, NoteHandlerSpeeds.STARBOARD_IDLE);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!sensorvalue) {
          currentState = State.INDEXING_FWD;
        }
        if (climbButton) {
          currentState = State.CLIMB;
        }
        if (ampAligning) {
          currentState = State.AMP_ALIGNING;
        }
        if (ampAligning && ampScoring) {
          currentState = State.AMP_SCORING_A;
        }
        if (ejectButton) {
          currentState = State.EJECT_DOWNSPOOL;
        }
        if (unjamButton) {
          currentState = State.UNJAM;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }

      break;

      case INDEXING_FWD:
        intake.runRollers(commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_INDEXING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_IDLE, NoteHandlerSpeeds.STARBOARD_IDLE);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (sensorvalue) {
          currentState = State.HOLDING;
        }
        if (climbButton) {
          currentState = State.CLIMB;
        }
        if (ampAligning) {
          currentState = State.AMP_ALIGNING;
        }
        if (ampAligning && ampScoring) {
          currentState = State.AMP_SCORING_A;
        }
        if (ejectButton) {
          currentState = State.EJECT_DOWNSPOOL;
        }
        if (unjamButton) {
          currentState = State.UNJAM;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }

      break;

      case HOLDING:
        intake.runRollers(commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_SHOOTER, NoteHandlerSpeeds.STARBOARD_SHOOTER);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!sensorvalue) {
          currentState = State.IDLE;
        }
        if (climbButton) {
          currentState = State.CLIMB;
        }
        if (ampAligning) {
          currentState = State.AMP_ALIGNING;
        }
        if (ampAligning && ampScoring) {
          currentState = State.AMP_SCORING_A;
        }
        if (ejectButton) {
          currentState = State.EJECT_DOWNSPOOL;
        }
        if (unjamButton) {
          currentState = State.UNJAM;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }

      break;

      case AMP_ALIGNING:
        intake.runRollers(commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
          : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_AMP_SCORE, NoteHandlerSpeeds.STARBOARD_AMP_SCORE);
        shooter.setArmAngle(ArmConstants.AMP_ALIGNMENT_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!ampAligning) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
        if (ampAligning && ampScoring) {
          currentState = State.AMP_SCORING_A;
        }
      
      break;

      case AMP_SCORING_A:
        intake.runRollers(commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_AMP_SCORE, NoteHandlerSpeeds.STARBOARD_AMP_SCORE);
        shooter.setArmAngle(ArmConstants.AMP_SCORE_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!ampScoring) {
          currentState = State.AMP_ALIGNING;
        }
        if (!ampAligning) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
        if (ampScoring && ampAligning && armInPosition) {
          currentState = State.AMP_SCORING_B;
        }

      break;
    
      case AMP_SCORING_B:
        intake.runRollers(commandedIntakeSpeed < 0 ? commandedIntakeSpeed : NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_AMP_SCORE, NoteHandlerSpeeds.STARBOARD_AMP_SCORE);
        shooter.setArmAngle(ArmConstants.AMP_SCORE_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!ampScoring) {
          currentState = State.AMP_ALIGNING;
        }
        if (!ampAligning) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      
      break;

      case HP_STATION_LOAD:
        intake.runRollers(NoteHandlerSpeeds.INTAKE_IDLE);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
        shooter.setShooterSpeed(NoteHandlerSpeeds.SHOOTER_INTAKE, NoteHandlerSpeeds.SHOOTER_INTAKE);
        shooter.setArmAngle(ArmConstants.HP_COLLECT_ANGLE);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!hpLoading) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (sensorvalue) {
          currentState = State.INDEXING_REV;
        }
        if (shooterbutton) {
          currentState = State.SPOOLING;
        }
        if (feederButton) {
          currentState = State.FEEDER_SPOOLING;
        }
        if (lowFeederButton) {
          currentState = State.LOW_FEEDER_SPOOLING;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      break;
    
      case EJECT_DOWNSPOOL:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE
            : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_EJECT, NoteHandlerSpeeds.STARBOARD_EJECT);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (shooterAtSpeed) {
          currentState = State.EJECT;
        }
        if (!ejectButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      break;
    
      case EJECT:
          intake.runRollers(commandedIntakeSpeed);
          shooter.runLoader(NoteHandlerSpeeds.LOADER_FIRING);
          shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_EJECT, NoteHandlerSpeeds.STARBOARD_EJECT);
          shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

          shooterAtSpeed = shooter.shootersAtSpeeds();
          armInPosition = shooter.armAtGoal();

          if (!ejectButton) {
            currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
          }
          if (autoShooting) {
            currentState = State.AUTO_SPOOLING;
          }
      break;

      case UNJAM:
        intake.runRollers(NoteHandlerSpeeds.INTAKE_UNJAM);
        shooter.runLoader(NoteHandlerSpeeds.LOADER_INTAKE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_IDLE, NoteHandlerSpeeds.STARBOARD_IDLE);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!unjamButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (sensorvalue) {
          currentState = State.INDEXING_REV;
        }
        if (ejectButton) {
          currentState = State.EJECT_DOWNSPOOL;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      break;

      case CLIMB:
        intake.runRollers(commandedIntakeSpeed);
        shooter.runLoader(commandedIntakeSpeed < 0 ? -NoteHandlerSpeeds.LOADER_INTAKE : NoteHandlerSpeeds.LOADER_IDLE);
        shooter.setShooterSpeed(NoteHandlerSpeeds.PORT_CLIMB, NoteHandlerSpeeds.STARBOARD_CLIMB);
        shooter.setArmAngle(ArmConstants.LOWER_LIMIT);

        shooterAtSpeed = shooter.shootersAtSpeeds();
        armInPosition = shooter.armAtGoal();

        if (!climbButton) {
          currentState = sensorvalue ? State.INDEXING_REV : State.IDLE;
        }
        if (autoShooting) {
          currentState = State.AUTO_SPOOLING;
        }
      break;
    }

    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
    
    SmartDashboard.putString("currentState", currentState.toString());
    SmartDashboard.putBoolean("sensor", sensorvalue);

    if ((debugFlags & CommandDebugFlags.NOTE_HANDLER) != 0) {
      SmartDashboard.putBoolean("ShooterAtSpeed", shooterAtSpeed);
      SmartDashboard.putBoolean("ChassisOnTarget", onTarget);
      SmartDashboard.putBoolean("ArmAtGoal", armInPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}