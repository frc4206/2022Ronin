// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticsSubsystem extends SubsystemBase {


  //needs the pistons for the harvestor and climber
  private DoubleSolenoid climberDSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.climberSolenoidFWD, Constants.Pneumatics.climberSolenoidBKWD);
  private DoubleSolenoid harvestorDSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.harvestorSolenoidFWD, Constants.Pneumatics.harvestorSolenoidBKWD);
  private DoubleSolenoid pancakeSolonoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.pankakeSolonoidFWD, Constants.Pneumatics.pankakeSolonoidBKWD);
  //private Solenoid test = new Solenoid(PneumaticsModuleType.REVPH, 4);
  //private PneumaticHub pneumaticHub = new PneumaticHub(23);

  //adds the sensors that we may use
  //private AnalogInput pneumaticPressureSensor = new AnalogInput(Constants.Pneumatics.pneumaticPressureSensor);
  //private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  
  public PneumaticsSubsystem() {
    //compressor.enableDigital();
  }

  public void dblSolenoid(DoubleSolenoid.Value direction) {
    climberDSolenoid.set(direction);
  }
  

  public String dblSolenoid_status(){
    return climberDSolenoid.get().toString();
  }


  public void GroundFeederShifter(){
    switch (harvestorDSolenoid.get()){
      case kOff:
        harvestorDSolenoid.set(DoubleSolenoid.Value.kForward);
       break;
      case kForward:
        harvestorDSolenoid.set(DoubleSolenoid.Value.kReverse);
       break;
      case kReverse:
        harvestorDSolenoid.set(DoubleSolenoid.Value.kForward);
        break;
    }
  }

  public void GroundFeederOut(){
    harvestorDSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void GroundFeederIn(){
    harvestorDSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void PancakeOut(){
    pancakeSolonoid.set(DoubleSolenoid.Value.kForward);
  }

  public void PancakeIn(){
    pancakeSolonoid.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //GlobalVariables.pneumaticPressureSensor = (250.0 * (pneumaticPressureSensor.getVoltage()/5.0)-25.0);
    //SmartDashboard.putString("Double Solenoid Status", dblSolenoid.get().toString());
  }
}
