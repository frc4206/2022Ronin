// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ClimberDownManualCommand extends CommandBase {
  private final ClimberSubsystem m_subsystem;
  private final PneumaticsSubsystem m_pneumatics;

  /** Creates a new Com_Motors. */
  public ClimberDownManualCommand(ClimberSubsystem subsystem, PneumaticsSubsystem pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_pneumatics = pneumatics;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumatics.PancakeIn();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.climber_down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.climber_stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
