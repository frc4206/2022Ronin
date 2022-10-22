// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PancakeOutCommand extends CommandBase {
  PneumaticsSubsystem m_pneumatics;
  public PancakeOutCommand(PneumaticsSubsystem pneumatics) {
     m_pneumatics = pneumatics;
    addRequirements( m_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumatics.PancakeOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
