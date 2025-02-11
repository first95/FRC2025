// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.format.ResolverStyle;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
 

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax winch;
  private final SparkMaxConfig winchConfig;
  public Climber() {
    winch = new SparkMax(ClimberConstants.CLIMBER_ID, MotorType.kBrushless);
    winchConfig = new SparkMaxConfig();

    winchConfig
      .inverted(ClimberConstants.INVERTED)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ClimberConstants.SMARTCURRENTLIMIT);
    
    winch.configure(winchConfig, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);

    
  }

  public void setClimberSpeed(double speed){
    winch.set(speed*ClimberConstants.MAX_SPEED);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command runWinch(double speed) {
    return new InstantCommand(() -> setClimberSpeed(speed));
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
