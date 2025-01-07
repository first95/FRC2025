// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CommandDebugFlags;

public class Climber extends SubsystemBase {
  private CANSparkMax winch, winch2;
  private int debugFlags;
  /** Creates a new Climber. */
  public Climber() {
    winch = new CANSparkMax(ClimberConstants.WINCH_ID, MotorType.kBrushless);
    winch2 = new CANSparkMax(ClimberConstants.WINCH2_ID, MotorType.kBrushless);

    winch.restoreFactoryDefaults();
    winch2.restoreFactoryDefaults();

    winch.setIdleMode(IdleMode.kBrake);
    winch2.setIdleMode(IdleMode.kBrake);

    winch.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT);
    winch2.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT);

    winch2.follow(winch, true);

    winch.setInverted(ClimberConstants.INVERT_WINCH);

    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);
  }

  public Command runWinches(double speed) {
    return new InstantCommand(() -> winch.set(speed));
  }

  @Override
  public void periodic() {
    debugFlags = (int) SmartDashboard.getNumber(CommandDebugFlags.FLAGS_KEY, 0);

    if ((debugFlags & ClimberConstants.DEBUG_FLAG) != 0) {
      // Put SmartDashboard debug here
    }
  }
}
