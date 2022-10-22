// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalVariables;

public class PowerHubSubsystem extends SubsystemBase {
  private PowerDistribution pdh = new PowerDistribution();
  /** Creates a new Subsys_Power_Hub. */
  public PowerHubSubsystem() {
    pdh.clearStickyFaults();
  }

  public double getCurrent(int pdhPort) {
    return pdh.getCurrent(pdhPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//    SmartDashboard.putString("Pneumatics System Pressure", GlobalVariables.pneumaticPressureSensor+"");
//    SmartDashboard.putString("Climber Left Amps", getCurrent(9)+"");
    SmartDashboard.putString("Climber Left Encoder", GlobalVariables.climberLeftEncoder+"");
//    SmartDashboard.putString("Climber Right Amps", getCurrent(8)+"");
    SmartDashboard.putString("Climber Right Encoder", GlobalVariables.climberRightEncoder+"");
//    SmartDashboard.putString("Climb Task", GlobalVariables.climbtasknumber+"");
//    SmartDashboard.putString("Limitswitch", GlobalVariables.Limitswitch+"");
//    SmartDashboard.putString("Upper Velocity", GlobalVariables.UpperShooterVelocity+"");
//    SmartDashboard.putString("Lower Velocity", GlobalVariables.LowerShooterVelocity+"");
//   SmartDashboard.putString("Upper Velocity Set", GlobalVariables.UpperVelocitySet+"");
//   SmartDashboard.putString("Lower Velocity Set", GlobalVariables.LowerVelocitySet+"");
  }
}
