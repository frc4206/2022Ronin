// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HarvestorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public HarvestorSubsystem() {
    IntakeMotor.setStatusFramePeriod(1, 20);
  }
  
  TalonFX IntakeMotor = new TalonFX(Constants.MotorsIDs.groundmotor, Constants.Canivore1);

  

  public void feederIn(){
    IntakeMotor.set(TalonFXControlMode.PercentOutput, Constants.MotorValues.FeederIn);
  }
  public void feederOut(){
    IntakeMotor.set(TalonFXControlMode.PercentOutput, Constants.MotorValues.FeederOut);
  }
  public void feederStop(){
    IntakeMotor.set(TalonFXControlMode.PercentOutput, Constants.MotorValues.FeederStop);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
  }
  
}
