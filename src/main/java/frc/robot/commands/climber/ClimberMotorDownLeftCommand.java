// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMotorDownLeftCommand extends CommandBase {
  private final ClimberSubsystem motors;
  private boolean isFinished;
  /** Creates a new Com_Motors. */
  public ClimberMotorDownLeftCommand(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    motors = subsystem;
    addRequirements(motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {/*
  if (motors.get_limitswitch() == false) {
    motors.climber_down();
  }
  else {
    isFinished = true;
    motors.climber_stop();
  }*/
  motors.climber_down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
