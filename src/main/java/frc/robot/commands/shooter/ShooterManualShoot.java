// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterManualShoot extends CommandBase {
  private final ShooterSubsystem m_shoot;
  /** Creates a new Com_Motors. */
  public ShooterManualShoot(ShooterSubsystem shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoot = shoot;
    addRequirements(m_shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shoot.shooterManualGo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoot.shooter_stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
