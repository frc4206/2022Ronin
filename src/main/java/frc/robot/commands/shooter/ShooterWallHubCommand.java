// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWallHubCommand extends CommandBase {
  private final ShooterSubsystem motors;
  /** Creates a new Com_Motors. */
  public ShooterWallHubCommand(ShooterSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    motors = subsystem;
    addRequirements(motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motors.shooterSetPowerWallHigh();
    motors.shooterWallHigh();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    motors.ledColor1();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
