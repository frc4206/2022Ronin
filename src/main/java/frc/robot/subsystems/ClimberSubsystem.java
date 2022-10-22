// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class ClimberSubsystem extends SubsystemBase {
  private WPI_TalonFX falconMotorR = new WPI_TalonFX(Constants.MotorsIDs.climberfalconMotorR, Constants.Canivore1);
  private WPI_TalonFX falconMotorL = new WPI_TalonFX(Constants.MotorsIDs.climberfalconMotorL, Constants.Canivore1);
  private DigitalInput Limitswitch = new DigitalInput(9);

  /** Creates a new Subsys_Motors. */
  public ClimberSubsystem() {
    falconMotorR.configFactoryDefault();
    falconMotorL.configFactoryDefault();
    falconMotorR.set(0.0);
    falconMotorL.set(0.0);
    falconMotorR.setInverted(true);
    falconMotorL.setInverted(false);
    falconMotorR.setNeutralMode(NeutralMode.Brake);
    falconMotorL.setNeutralMode(NeutralMode.Brake);
    falconMotorR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor.RemoteSensor0, 0, 30); //Local Feedback Source
    falconMotorL.configRemoteFeedbackFilter(falconMotorR.getDeviceID(), RemoteSensorSource.TalonFX_SelectedSensor, 0, 30);
    falconMotorL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    
 
/* set deadband to super small 0.001 (0.1 %).
  The default deadband is 0.04 (4 %) */
falconMotorL.configNeutralDeadband(0.001, 30);
falconMotorR.configNeutralDeadband(0.001, 30);

/*
 * Talon FX does not need sensor phase set for its integrated sensor
 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
 * 
 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
 */
    // _talon.setSensorPhase(true);

/* Set relevant frame periods to be at least as fast as periodic rate */
falconMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
falconMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, 30);
falconMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
falconMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, 30);

/* Set the peak and nominal outputs */
falconMotorL.configNominalOutputForward(0, 30);
falconMotorL.configNominalOutputReverse(0, 30);
falconMotorL.configPeakOutputForward(0.5, 30);
falconMotorL.configPeakOutputReverse(-0.5, 30);

/* Set Motion Magic gains in slot0 - see documentation */
falconMotorL.selectProfileSlot(0, 0);
falconMotorL.config_kF(0, 3.0, 30);
falconMotorL.config_kP(0, 0.46, 30);
falconMotorL.config_kI(0, 0.001, 30);
falconMotorL.config_kD(0, 4.6, 30);

/* Set acceleration and vcruise velocity - see documentation */
falconMotorL.configMotionCruiseVelocity(7500, 30);
falconMotorL.configMotionAcceleration(3000, 30);
falconMotorL.configMotionSCurveStrength(3);
falconMotorR.configFactoryDefault();
falconMotorL.configFactoryDefault();
falconMotorR.setNeutralMode(NeutralMode.Brake);
falconMotorL.setNeutralMode(NeutralMode.Brake);
falconMotorR.follow(falconMotorL);

falconMotorL.setStatusFramePeriod(1, 20);
falconMotorR.setStatusFramePeriod(1, 20);

//                                                                       enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  
falconMotorL.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,      85,                90,                1.0));
falconMotorR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,      85,                90,                1.0));

}

  public void climber_up() {
    falconMotorL.set(0.8);
  }

  public void climber_down() {
    falconMotorL.set(-0.8);
  }

  public void climber_stop() {
    falconMotorL.set(ControlMode.PercentOutput, 0.0);
  }

  public void motion_magic(double position) {
    falconMotorL.set(ControlMode.MotionMagic, position, DemandType.AuxPID, 0.0); 
    }

  public void initialize_enconders() {
    falconMotorR.setSelectedSensorPosition(0, 0, 30);
    falconMotorL.setSelectedSensorPosition(0, 0, 30);
  }

  public double get_encoders() {
    return falconMotorL.getSelectedSensorPosition();
  }

  public void set_climb_task(String task_number) {
    GlobalVariables.climbtasknumber = task_number;
  }

  public String get_climb_task() {
    return GlobalVariables.climbtasknumber;
  }

  public boolean get_limitswitch() {
    return Limitswitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // GlobalVariables.climberLeftEncoder = falconMotorL.getSelectedSensorPosition();
    // GlobalVariables.climberRightEncoder = falconMotorR.getSelectedSensorPosition();
    // GlobalVariables.Limitswitch = Limitswitch.get();
  }
}
