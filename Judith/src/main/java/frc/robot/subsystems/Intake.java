// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax topIntakeRoller, bottomIntakeRoller;

  private int debugFlags;
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    topIntakeRoller = new CANSparkMax(IntakeConstants.TOP_ROLLER_ID, MotorType.kBrushless);
    bottomIntakeRoller = new CANSparkMax(IntakeConstants.BOTTOM_ROLLER_ID, MotorType.kBrushless);

    topIntakeRoller.restoreFactoryDefaults();
    bottomIntakeRoller.restoreFactoryDefaults();

    topIntakeRoller.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    bottomIntakeRoller.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, IntakeConstants.STATUS_FRAME_0_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, IntakeConstants.STATUS_FRAME_1_PERIOD);
    topIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, IntakeConstants.STATUS_FRAME_2_PERIOD);

    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, IntakeConstants.STATUS_FRAME_0_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, IntakeConstants.STATUS_FRAME_1_PERIOD);
    bottomIntakeRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, IntakeConstants.STATUS_FRAME_2_PERIOD);

    topIntakeRoller.burnFlash();
    bottomIntakeRoller.burnFlash();

    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
  }

  public void runRollers(double speed) {
    topIntakeRoller.set(speed * IntakeConstants.MAX_SPEED);
    bottomIntakeRoller.set(speed * IntakeConstants.MAX_SPEED);
  }

  @Override
  public void periodic() {
    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
    
    if ((debugFlags & IntakeConstants.DEBUG_FLAG) != 0) {
      // Put SmartDashboard printouts here
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
