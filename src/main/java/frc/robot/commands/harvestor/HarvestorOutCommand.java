// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvestor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class HarvestorOutCommand extends CommandBase {
  private double startmotortime;
  HarvestorSubsystem m_harvestor;
  PneumaticsSubsystem m_pneumatics;
  public HarvestorOutCommand(HarvestorSubsystem harvestor, PneumaticsSubsystem pneumatics) {
     m_harvestor = harvestor ;
     m_pneumatics = pneumatics;
    addRequirements(m_harvestor, m_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    m_pneumatics.GroundFeederOut();
    startmotortime = Timer.getFPGATimestamp() + .5;
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_harvestor.feederIn();
    if(Timer.getFPGATimestamp() > startmotortime){
      m_harvestor.feederIn();
    }
    //m_pneumatics.testRun1();
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
