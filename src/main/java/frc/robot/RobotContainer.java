// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.AxisTrigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.climber.AngleClimbCommand;
import frc.robot.commands.climber.ClimberDownManualCommand;
import frc.robot.commands.climber.ClimberUpManualCommand;
import frc.robot.commands.climber.PancakeInCommand;
import frc.robot.commands.climber.PancakeOutCommand;
import frc.robot.commands.conveyor.ConveyorAutoCommand;
import frc.robot.commands.conveyor.ConveyorBackwardCommand;
import frc.robot.commands.conveyor.ConveyorForwardCommand;
import frc.robot.commands.harvestor.HarvestorInCommand;
import frc.robot.commands.harvestor.HarvestorOutCommand;
import frc.robot.commands.harvestor.HarvestorReverseCommand;
import frc.robot.commands.harvestor.HarvestorStopCommand;
import frc.robot.commands.shooter.ShooterHubLowerCommand;
import frc.robot.commands.shooter.ShooterStopCommand;
import frc.robot.commands.shooter.ShooterWallHubCommand;
import frc.robot.commands.shooter.ShooterWallHubPlusCommand;
import frc.robot.commands.shooter.ShooterXSpotCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);


  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ClimberSubsystem motors = new ClimberSubsystem();
  private final PneumaticsSubsystem pneumatics = new PneumaticsSubsystem();
  private final HarvestorSubsystem harvestor = new HarvestorSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ConveyorSubsystem conveyor = new ConveyorSubsystem();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    //makes the smartdashboard f0r auto commands
    //autoChooser.addOption("S Then Backwards", new exampleAuto(swerve));
    //autoChooser.addOption("DriveForwardOnly", new DriveForawrdAuto(swerve));
    //autoChooser.addOption("TwoBallRightForward", new TwoBallRightForwardAuto(swerve));
    //autoChooser.addOption("ThreeBallTerminal", new ThreeBallTearminalAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("ThreeBallHub", new ThreeBallHubAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("Kevins3to5BallFTWorth", new Kevins3to5BallOriganalAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("Kevins3to5BallShortEnd", new Kevins3to5BallShortEndAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("Andrews3to5BallShortAll", new Andrews3to5BallShortAllAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("Andrews4Ball", new Andrews4BallShortAllAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("Kevins3to5BallShortAll", new Kevins3to5BallShortAllAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("BackupAndShoot", new BackupAndShootAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("TwoBallLeft", new TwoBallLeftAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("TwoBallLeftUpgrade", new TwoBallLeftUpgradeAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    autoChooser.addOption("TwoBallLeftKick", new TwoBallLeftKickAuto(swerve, harvestor, conveyor, shooter, pneumatics));
    SmartDashboard.putData("Auto Selector", autoChooser);
    
    
    // Configure the button bindings
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    //-----------------------Driver Buttons----------------------------------------------/
    zeroGyro.whenPressed(new InstantCommand(() -> swerve.zeroGyro()));
    new JoystickButton(driver, XboxController.Button.kA.value).whileHeld(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, false, true));
    //new JoystickButton(driver, XboxController.Button.kB.value).whileHeld(new TeleopSwerve(swerve, driver, 0, 0, rotationAxis, true, true));
    new JoystickButton(driver, XboxController.Button.kB.value).whileHeld(new VisionAlignStopCommand(swerve, true,true));




    //-----------------------Shooter Buttons----------------------------------------------/
    new JoystickButton(driver, Button.kLeftBumper.value).whenPressed(new ShooterWallHubCommand(shooter));
    //new JoystickButton(driver, Button.kRightBumper.value).whenPressed(new ShooterXSpotCommand(shooter));
    new JoystickButton(driver, Button.kRightBumper.value).whenPressed(new ShooterHubLowerCommand(shooter));

    new AxisTrigger(driver, 3).whenPressed(new ShooterStopCommand(shooter));
    new AxisTrigger(driver, 2).whenPressed(new ShooterStopCommand(shooter));




    //-----------------------Climbing Buttons----------------------------------------------/
    //more specific button combinations for complex programs
    //new JoystickButton(operator, Button.kY.value).whenPressed(new AutoClimbCommand(pneumatics, motors));
    //new JoystickButton(operator, Button.kB.value).whenPressed(new ClimberMotorUpCommand(motors));
    //new JoystickButton(operator, Button.kA.value).whenPressed(new ClimberMotorDownCommand(motors));
    //new JoystickButton(operator, Button.kX.value).whenPressed(new ClimberMotorStopCommand(motors));

    //basic up and down movement that is manual buttons, run by co-driver
    new JoystickButton(operator, Button.kLeftStick.value).whileHeld(new ClimberDownManualCommand(motors, pneumatics));
    new JoystickButton(operator, Button.kRightStick.value).whileHeld(new ClimberUpManualCommand(motors, pneumatics));
    new JoystickButton(operator, Button.kStart.value).whenPressed(new PancakeInCommand(pneumatics));
    new JoystickButton(operator, Button.kBack.value).whenPressed(new PancakeOutCommand(pneumatics));


    //driver gets the pison command for now 
    new JoystickButton(driver, Button.kBack.value).whenPressed(new AngleClimbCommand(pneumatics));



    //-----------------------Harvestor Buttons----------------------------------------------/
    new AxisTrigger(operator, 2).whileHeld(new HarvestorOutCommand(harvestor, pneumatics));
    new AxisTrigger(operator, 3).whenPressed(new HarvestorInCommand(harvestor, pneumatics));
    new JoystickButton(operator, Button.kLeftBumper.value).whenPressed(new HarvestorStopCommand(harvestor));

    new JoystickButton(operator, Button.kRightBumper.value).whileHeld(new HarvestorReverseCommand(harvestor));
    


    //-----------------------Conveyor Buttons----------------------------------------------/
    new JoystickButton(operator, Button.kA.value).whileHeld(new ConveyorForwardCommand(conveyor));
    new JoystickButton(operator, Button.kB.value).whileHeld(new ConveyorBackwardCommand(conveyor));
    new JoystickButton(operator, Button.kY.value).whenPressed(new ConveyorAutoCommand(conveyor));


    //in case or driver wanting to shoot
    new JoystickButton(driver, Button.kY.value).whileHeld(new ConveyorForwardCommand(conveyor));  
  }


  public void setRumble(){

    driver.setRumble(RumbleType.kLeftRumble, 1);
    driver.setRumble(RumbleType.kRightRumble, 1);
  }

  public void offRumble(){
    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);
  }


  public Command getAutonomousCommand() {
    //goes to the auto chooser selction
    return autoChooser.getSelected();
  }
}