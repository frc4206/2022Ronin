// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AngleClimbCommand extends CommandBase {
  private final PneumaticsSubsystem pneumatics;
  /** Creates a new Com_AngleClimb. */
  public AngleClimbCommand(PneumaticsSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumatics = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (pneumatics.dblSolenoid_status()){
      case "kOff":
        pneumatics.dblSolenoid(DoubleSolenoid.Value.kForward);
        break;
      case "kForward":
        pneumatics.dblSolenoid(DoubleSolenoid.Value.kReverse);
        break;
      case "kReverse":
        pneumatics.dblSolenoid(DoubleSolenoid.Value.kForward);
    }
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
    return true;
  }
}
