// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CommandDebugFlags;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.L1ArmConstants;
import frc.robot.Constants.L4ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Vision;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralHandlerCommand;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.commands.drivebase.AbsoluteDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.L1Arm;
import frc.robot.subsystems.L4Arm;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.SwerveBase;

import static edu.wpi.first.units.Units.Rotation;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.sql.Driver;
import java.time.Instant;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import java.util.Arrays;


import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Map<String, Optional<Trajectory<SwerveSample>>> trajMap;

  // The robot's subsystems and commands are defined here...
  private final SwerveBase drivebase = new SwerveBase();
  private final L1Arm L1arm = new L1Arm();
  private final L4Arm L4arm = new L4Arm();
  private final Climber climber = new Climber();
  //private final TeleopDrive openRobotRel, closedRobotRel, openFieldRel, closedFieldRel;
  private final AbsoluteDrive absoluteDrive;
  private final CoralHandlerCommand coralHandler;

  private final CommandJoystick driveController = new CommandJoystick(OperatorConstants.driveControllerPort);
  private final CommandJoystick headingController = new CommandJoystick(OperatorConstants.headingControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.operatorControllerPort);
  
  private Command autoCommand;
  public AutoChooser autoChooser;
  private final SendableChooser<String> modularAutoTargetChooser = new SendableChooser<>();
  private String[] modularAutoTargets = {};
  String currentModularAuto = "";
  private final Autos autos;
  

  SendableChooser<Integer> debugMode = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    /*openRobotRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
            ? -driveController.getY()
            : 0,
        () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
            ? -driveController.getX()
            : 0,
        () -> -headingController.getTwist(), () -> false, true);

    closedRobotRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
            ? -driveController.getY()
            : 0,
        () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
            ? -driveController.getX()
            : 0,
        () -> -headingController.getTwist(), () -> false, false);

    openFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
            ? driveController.getY()
            : 0,
        () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
            ? driveController.getX()
            : 0,
        () -> -headingController.getTwist(), () -> true, true);

    closedFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
            ? -driveController.getY()
            : 0,
        () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
            ? -driveController.getX()
            : 0,
        () -> -headingController.getTwist(), () -> true, false);*/

    absoluteDrive = new AbsoluteDrive(
        drivebase,
        () -> (Math.abs(driveController.getY()) > OperatorConstants.joystickDeadband)
            ? -driveController.getY()
            : 0,
        () -> (Math.abs(driveController.getX()) > OperatorConstants.joystickDeadband)
            ? -driveController.getX()
            : 0,
        () -> -headingController.getX(),
        () -> -headingController.getY(),
        false,
        () -> driveController.getHID().getRawButton(3));

        drivebase.setDefaultCommand(absoluteDrive);


    coralHandler = new CoralHandlerCommand(
      () -> operatorController.getHID().getLeftBumperButton(),    // L1Intake
      () -> operatorController.getHID().getAButton(),    // L1Eject
      () -> operatorController.getHID().getYButton(),    //L1Inject
      () -> driveController.getHID().getRawButton(1),   // L4 intake
      () -> headingController.getHID().getRawButton(1), // L4 Score
      () -> operatorController.getHID().getBButton(),   // Handoff
      () -> operatorController.getHID().getRightBumperButton(), //L1Score
      () -> operatorController.getHID().getLeftStickButton(), //Stow
      () -> operatorController.getHID().getXButton(), //L1 HumanLoading
      () -> headingController.getHID().getRawButton(2), // point to Reef
      () -> driveController.getHID().getRawButton(2),//auto align with humanLoad
      () -> operatorController.povUp().getAsBoolean(),//Climber Out
      () -> operatorController.getHID().getRightStickButton(),//ignore L1 arm Position
      L1arm,
      L4arm,
      climber,
      drivebase,
      absoluteDrive
    );
    
    L1arm.setDefaultCommand(coralHandler);
    L4arm.setDefaultCommand(coralHandler);

    // Configure the trigger bindings
    configureBindings();

    trajMap = loadTrajectories();

    SmartDashboard.putNumber("KV", Drivebase.KV);
    SmartDashboard.putNumber("KA", Drivebase.KA);
    SmartDashboard.putNumber("KP", 0);
    SmartDashboard.putNumber("KI", 0);
    SmartDashboard.putNumber("KD", 0);

    SmartDashboard.putNumber("shoulderKP", L4ArmConstants.KP);
    SmartDashboard.putNumber("shoulderKI", 0);
    SmartDashboard.putNumber("shoulderKD", 0);
    SmartDashboard.putNumber("shoulderKF", 0);

    SmartDashboard.putNumber("shoulderKS",  0);
    SmartDashboard.putNumber("shoulderKG", L4ArmConstants.KG);
    SmartDashboard.putNumber("shoulderKV", 0);
    SmartDashboard.putNumber("shoulderKA", 0);

    SmartDashboard.putNumber("setShoulderAngleNumber", 0);

    
    autoChooser = new AutoChooser();
    autos = new Autos(drivebase,L4arm);

    autoChooser.addRoutine("ModularAuto",autos :: ModularAuto);
    autoChooser.addRoutine("threeCoralAutoLeftSide", autos :: threeCoralAutoLeftSide);
    autoChooser.addRoutine("threeCoralAutoRightSide", autos::threeCoralAutoRightSide);

    autoChooser.select("ModularAuto");
    //autoChooser.addRoutine("Example Routine", this::exampleRoutine);
    //autoChooser.addCmd("Example Auto Command", this::exampleAutoCommand);

    modularAutoTargetChooser.addOption("S0", "S0");
    modularAutoTargetChooser.addOption("S1", "S1");
    modularAutoTargetChooser.addOption("S2", "S2");
    modularAutoTargetChooser.addOption("R00", "R00");
    modularAutoTargetChooser.addOption("R01", "R01");
    modularAutoTargetChooser.addOption("R10", "R10");
    modularAutoTargetChooser.addOption("R11", "R11");
    modularAutoTargetChooser.addOption("R20", "R20");
    modularAutoTargetChooser.addOption("R21", "R21");
    modularAutoTargetChooser.addOption("R30", "R30");
    modularAutoTargetChooser.addOption("R31", "R31");
    modularAutoTargetChooser.addOption("R40", "R40");
    modularAutoTargetChooser.addOption("R41", "R41");
    modularAutoTargetChooser.addOption("R50", "R50");
    modularAutoTargetChooser.addOption("R51", "R51");
    modularAutoTargetChooser.addOption("L0", "L0");
    modularAutoTargetChooser.addOption("L1", "L1");
    
    SmartDashboard.putData("autoChooser",autoChooser);
    
    SmartDashboard.putData("modularAutoChooser",modularAutoTargetChooser);
    SmartDashboard.putString("currentModularAuto", "");
    
    SmartDashboard.putData("setShoulderGains",
    new InstantCommand(
      () -> L4arm.setGains()
    ).ignoringDisable(true));
    
    SmartDashboard.putData("addPosToAuto",
      new InstantCommand(
        () -> addToModularAuto()
      )
      .ignoringDisable(true)
      );

    SmartDashboard.putData("setArmAngle",
      new InstantCommand(
        () -> L4arm.setArmAngle(Rotation2d.fromDegrees(0))
      ).ignoringDisable(true));
    
    SmartDashboard.putData("removePosFromAuto",
      new InstantCommand(
        () -> removeFromModularAuto()
    )
    .ignoringDisable(true)
    );

    SmartDashboard.putData("setGains", new InstantCommand(drivebase::setVelocityModuleGains));
    SmartDashboard.putData("SendAlliance",
      new InstantCommand(
        () -> drivebase.setAlliance(DriverStation.getAlliance().get())
      )
      .ignoringDisable(true)
    );
    SmartDashboard.putBoolean("bool", false);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    //operatorController.start().onTrue(new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(new Translation2d(),new Rotation2d()))).ignoringDisable(true));
    operatorController.start().onTrue(new InstantCommand(drivebase::clearOdometrySeed).ignoringDisable(true));
    driveController.button(8).onTrue(new InstantCommand(drivebase::clearOdometrySeed).ignoringDisable(true));
    operatorController.povDown().onTrue(climber.runWinch(ClimberConstants.WINCH_IN_SPEED));
    operatorController.povUp().onTrue(climber.runWinch(ClimberConstants.WINCH_OUT_SPEED));
    operatorController.povCenter().onTrue(climber.runWinch(0));
    headingController.button(3).and(coralHandler.getCoralInL1())
      .whileTrue(coralHandler.LefthandOffAndL4())
      .onFalse(coralHandler.cancelL4AutoScore());
    headingController.button(3).and(coralHandler.getCoralNotInL1())
      .whileTrue(coralHandler.LeftL4AutoScore())
      .onFalse(coralHandler.cancelL4AutoScore());
    headingController.button(4).and(coralHandler.getCoralInL1())
      .whileTrue(coralHandler.RighthandOffAndL4())
      .onFalse(coralHandler.cancelL4AutoScore());
    headingController.button(4).and(coralHandler.getCoralNotInL1())
      .whileTrue(coralHandler.RightL4AutoScore())
      .onFalse(coralHandler.cancelL4AutoScore());
    driveController.button(4)
      .whileTrue(coralHandler.L1AutoScore())
      .onFalse(coralHandler.cancelL1AutoScore());
    operatorController.button(9).onTrue(new InstantCommand(() -> setBrakes(false)).ignoringDisable(true));
    // operatorController.button(5).whileTrue(
    //   new AlignToPose("Reef", drivebase)//align to scoring position
    //   .andThen(new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble,1)))//when aligned vibrate the controller
    // )
    // .onFalse(new InstantCommand(() -> operatorController.setRumble(RumbleType.kBothRumble, 0)));
    //   L1arm.incrementArmVoltage(0.01);   
    // }  

    // if (operatorController.b().getAsBoolean() == true){
    //   L1arm.incrementArmVoltage(-0.01);
    // }
    // operatorController.a().whileTrue(new InstantCommand(() -> L1arm.incrementArmVoltage(0.001)));
    // operatorController.b().whileTrue(new InstantCommand(() -> L1arm.incrementArmVoltage(-0.001)));
    // operatorController.a().whileTrue(l4arm.sysIdDynShoulder(SysIdRoutine.Direction.kForward));
    // operatorController.b().whileTrue(l4arm.sysIdDynShoulder(SysIdRoutine.Direction.kReverse));
    // operatorController.x().whileTrue(l4arm.sysIdQuasiShoulder(SysIdRoutine.Direction.kForward));
    // operatorController.y().whileTrue(l4arm.sysIdQuasiShoulder(SysIdRoutine.Direction.kReverse));
    /*driveController.button(2).whileTrue(new AutoAmp(drivebase)).onFalse(new InstantCommand(() -> {
      SmartDashboard.putBoolean(Auton.AUTO_AMP_SCORE_KEY, false);
      SmartDashboard.putBoolean(Auton.AUTO_AMP_ALIGN_KEY, falSse);
    }));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.selectedCommand();
  }
  
  public void addToModularAuto(){
    final int N = modularAutoTargets.length;
    modularAutoTargets = Arrays.copyOf(modularAutoTargets,N + 1);
    modularAutoTargets[N] = modularAutoTargetChooser.getSelected();
    currentModularAuto = "";
    for(String posTarget : modularAutoTargets){
      currentModularAuto += posTarget + ",";
    }
    SmartDashboard.putStringArray("modularAutoTargets", modularAutoTargets);
    SmartDashboard.putString("currentModularAuto",currentModularAuto);
  }
  public void removeFromModularAuto(){
    final int N = modularAutoTargets.length;
    if (N > 0){
      modularAutoTargets = Arrays.copyOf(modularAutoTargets,N - 1);
      currentModularAuto = "";
      for(String posTarget : modularAutoTargets){
        currentModularAuto += posTarget + ",";
      }
      
      SmartDashboard.putStringArray("modularAutoTargets", modularAutoTargets);
      SmartDashboard.putString("currentModularAuto", currentModularAuto);
    }
  }
  

  public void stopDrive() {
    drivebase.setChassisSpeeds(new ChassisSpeeds());
  }

  public void sendAlliance() {
    drivebase.setAlliance(DriverStation.getAlliance().get());
  }

  public void setIsAuto(boolean isAuto) {
    drivebase.isAuto = isAuto;
    SmartDashboard.putBoolean(Constants.Auton.AUTO_ENABLED_KEY, isAuto);
    
  }

  public void setBrakes(boolean setbrake){
    L1arm.setBrake(setbrake);
    L4arm.setBrake(setbrake);
    drivebase.setMotorBrake(setbrake);
  }

  private Map<String, Optional<Trajectory<SwerveSample>>> loadTrajectories() {
    Set<String> trajNames;
    try {
      if (Robot.isReal()) {
        trajNames = listFilesUsingFilesList("/home/lvuser/deploy/choreo");
      } else {
        trajNames = listFilesUsingFilesList("src/main/deploy/choreo");
      }
    } catch (IOException e) {
      DriverStation.reportError("Invalid Directory! Trajectories failed to load!", true);
      return null;
    }
    return trajNames.stream().collect(Collectors.toMap(
        entry -> entry.replace(".traj", ""),
        entry -> Choreo.loadTrajectory(entry.replace(".traj", ""))));
  }

  private Set<String> listFilesUsingFilesList(String dir) throws IOException {
    try (Stream<Path> stream = Files.list(Paths.get(dir))) {
      return stream
          .filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .collect(Collectors.toSet());
    }
  }
}
