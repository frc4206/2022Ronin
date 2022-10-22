// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSubsystem extends SubsystemBase {
  private WPI_TalonFX shooterUpper = new WPI_TalonFX(Constants.MotorsIDs.shooterUpperMotor, Constants.Canivore1);
  private WPI_TalonFX shooterLower = new WPI_TalonFX(Constants.MotorsIDs.shooterLowerMotor, Constants.Canivore1);
  private Spark blinkinLED = new Spark(3);

  private double velocitysetHighXSpot = 0.0;
  private double velocitysetLowXSpot = 0.0;

  private double velocitysetHighWallHub = 0.0;
  private double velocitysetLowWallHub = 0.0;

  private double velocitysetHighWallHubSemiplus = 0.0;
  private double velocitysetLowWallHubSemiplus = 0.0;

  private double velocitysetHighWallHubplus = 0.0;
  private double velocitysetLowWallHubplus = 0.0;

  private double velocitysetHighWallLowHub = 0.0;
  private double velocitysetLowWallLowHub = 0.0;



  private boolean shooterAtSpeed = false;

  public boolean getShooterSpeed(){
    return shooterAtSpeed;
  }

  public void setShooterSpeed(boolean shooterAtSpeedSet){
    shooterAtSpeed = shooterAtSpeedSet;
  }

  public ShooterSubsystem() {
    shooterUpper.configFactoryDefault();
    shooterLower.configFactoryDefault();
    shooterUpper.set(ControlMode.PercentOutput, 0.0);
    shooterLower.set(ControlMode.PercentOutput, 0.0);
    shooterLower.setInverted(true);
    shooterUpper.setInverted(false);
    shooterUpper.setNeutralMode(NeutralMode.Coast);
    shooterLower.setNeutralMode(NeutralMode.Coast);
   
      shooterUpper.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
      shooterLower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    
  
    //Name: Brennan 
    //About: configure the flywheels and set the PID loops(create the horizontal asympote for the velocity to get to) for the most optimal velocity control  
  
      shooterUpper.configClosedloopRamp(0.5, 30);
      shooterLower.configClosedloopRamp(0.5, 30);
      shooterUpper.configAllowableClosedloopError(0, 10);
      shooterLower.configAllowableClosedloopError(0, 10);

      shooterUpper.selectProfileSlot(0, 0);
      shooterUpper.config_kF(0, 0.049, 30);
      shooterUpper.config_kP(0, 0.055, 30);
      shooterUpper.config_kI(0, 0.0005, 30);
      shooterUpper.config_kD(0, 20.0, 30);
      shooterLower.selectProfileSlot(0, 0);
      shooterLower.config_kF(0, 0.049, 30);
      shooterLower.config_kP(0, 0.055, 30);
      shooterLower.config_kI(0, 0.0005, 30);
      shooterLower.config_kD(0, 20.0, 30);

      shooterUpper.configNominalOutputForward(0.0, 30);
      shooterUpper.configNominalOutputReverse(0.0, 30);
    
      shooterLower.configNominalOutputForward(0.0, 30);
      shooterLower.configNominalOutputReverse(0.0, 30);

      shooterLower.setStatusFramePeriod(21, 1000);
      shooterUpper.setStatusFramePeriod(21, 1000);
      shooterLower.setStatusFramePeriod(1, 20);
      shooterUpper.setStatusFramePeriod(1, 20);

    
  
  }

  public double getShooterVelo(){
    return shooterLower.getSelectedSensorVelocity();
  }

  public void shooterBolean(){
    if(getShooterVelo() >= 8050){
      shooterAtSpeed = true;
    }
    else{
      shooterAtSpeed = false;
    }
  }


  //set power for diffrent places
  public void shooterSetPowerXSpot() {
    velocitysetHighXSpot = GlobalVariables.UpperVelocitySetXSpot;
    velocitysetLowXSpot = GlobalVariables.LowerVelocitySetXSpot;
  }

  public void shooterSetPowerWallHigh() {
    velocitysetHighWallHub = GlobalVariables.UpperVelocitySetWallHub;
    velocitysetLowWallHub = GlobalVariables.LowerVelocitySetWallHub;
  }

  public void shooterSetPowerWallHighPlus() {
    velocitysetHighWallHubplus = GlobalVariables.UpperVelocitySetWallHubplus;
    velocitysetLowWallHubplus = GlobalVariables.LowerVelocitySetWallHubplus;
  }

  public void shooterSetPowerWallHighSemiPlus() {
    velocitysetHighWallHubSemiplus = GlobalVariables.UpperVelocitySetWallHubsemiplus;
    velocitysetLowWallHubSemiplus = GlobalVariables.LowerVelocitySetWallHubsemiplus;
  }
  
  public void shooterSetPowerWallLow() {
    velocitysetHighWallLowHub = GlobalVariables.UpperVelocitySetWallLow;
    velocitysetLowWallLowHub = GlobalVariables.LowerVelocitySetWallLow;
  }


  //shooting coomads
  public void shooterXSpotHub() {
    shooterUpper.set(ControlMode.Velocity, velocitysetHighXSpot);
    shooterLower.set(ControlMode.Velocity, velocitysetLowXSpot);
  }

  public void shooterWallHigh() {
    shooterUpper.set(ControlMode.Velocity, velocitysetHighWallHub);
    shooterLower.set(ControlMode.Velocity, velocitysetLowWallHub);
  }

  public void shooterWallHighPlus() {
    shooterUpper.set(ControlMode.Velocity, velocitysetHighWallHubplus);
    shooterLower.set(ControlMode.Velocity, velocitysetLowWallHubplus);
  }

  public void shooterWallHighSemiPlus() {
    shooterUpper.set(ControlMode.Velocity, velocitysetHighWallHubSemiplus);
    shooterLower.set(ControlMode.Velocity, velocitysetLowWallHubSemiplus);
  }

  public void shooterWallLow() {
    shooterUpper.set(ControlMode.Velocity, velocitysetHighWallLowHub);
    shooterLower.set(ControlMode.Velocity, velocitysetLowWallLowHub);
  }

  public void shooter_stop() {
    shooterUpper.set(ControlMode.PercentOutput, 0.0);
    shooterLower.set(ControlMode.PercentOutput, 0.0);
  }

  public void shooterManualGo() {
    shooterUpper.set(ControlMode.PercentOutput, Constants.MotorValues.shooteruppermanual);
    shooterLower.set(ControlMode.PercentOutput, Constants.MotorValues.shooterlowermanual);
  }

  public void initialize_enconders() {
    shooterUpper.setSelectedSensorPosition(0, 0, 30);
    shooterLower.setSelectedSensorPosition(0, 0, 30);
  }

  public void ledColor1(){
    blinkinLED.set(0.85);//-0.09blue flash
                          //-0.51ocean pattern blue
  }

  public void ledColor2(){
    blinkinLED.set(0.73);
  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    //Global_Variables.UpperShooterPower = falconShooterUpper.getMotorOutputPercent();
    //Global_Variables.LowerShooterPower = falconShooterLower.getMotorOutputPercent();
    // GlobalVariables.UpperShooterVelocity = falconShooterUpper.getSelectedSensorVelocity();
    // GlobalVariables.LowerShooterVelocity = falconShooterLower.getSelectedSensorVelocity();


     SmartDashboard.putString("UpperShooterRPM", shooterUpper.getSelectedSensorVelocity()+"");
     SmartDashboard.putString("LowerShooterRPM", shooterLower.getSelectedSensorVelocity()+"");

  }
}
