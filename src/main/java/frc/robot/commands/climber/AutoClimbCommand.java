// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AutoClimbCommand extends CommandBase {
  private final PneumaticsSubsystem pneumatics;
  private final ClimberSubsystem motors;
  private String climbtask;
  /** Creates a new Com_AutoClimb. */
  public AutoClimbCommand( PneumaticsSubsystem pneumatic, ClimberSubsystem motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    pneumatics = pneumatic;
    motors = motor;
    addRequirements(pneumatics, motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motors.initialize_enconders();
    motors.set_climb_task("1-extend climber(1st bar)");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbtask = motors.get_climb_task();

    switch (climbtask) {
      case "1-extend climber(1st bar)":
        motors.motion_magic(2000000);
        motors.set_climb_task("1A-climber extended");
        break;
      case "1A-climber extended":
        if(motors.get_encoders() >= 1000000) {
          motors.set_climb_task("2-retract climber(1st bar)");
        }
        break;
      case "2-retract climber(1st bar)":
        motors.motion_magic(0);
        motors.set_climb_task("2A-climber retracted");
        break;
      case "2A-climber retracted":  
        if(motors.get_encoders() <= 10) {
          motors.set_climb_task("3-tilt climber(2nd bar)");
        }
        if(motors.get_limitswitch()) {
          motors.climber_stop();
          motors.set_climb_task("3-tilt climber(2nd bar)");
        }
        break;
      case "3-tilt climber(2nd bar)":
        if(motors.get_limitswitch() == false) {
        motors.motion_magic(2000000);
        motors.set_climb_task("3A-climber extended");
        }
        break;
      case "3A-climber extended":
        if(motors.get_encoders() > 200000) {
          pneumatics.dblSolenoid(DoubleSolenoid.Value.kForward);
          motors.set_climb_task("3B-extend climber(2nd bar)"); 
        }
        break;
      case "3B-extend climber(2nd bar)":
        if(motors.get_encoders() > 1000000) {
          motors.set_climb_task("5-retract climber(2nd bar)");
        } 
        break;
      case "5-retract climber(2nd bar)":
        pneumatics.dblSolenoid(DoubleSolenoid.Value.kReverse);   
        motors.motion_magic(0);
        motors.set_climb_task("5A-climber retracted");
        break;
      case "5A-climber retracted":
        if(motors.get_encoders() <= 10) {
          motors.climber_stop();
          motors.set_climb_task("(test)6-tilt climber(3rd bar)");
        }
        break;
      case "6-tilt climber(3rd bar)":
        motors.climber_down();
        if(motors.get_encoders() > 500000) {
          motors.climber_stop();
          pneumatics.dblSolenoid(DoubleSolenoid.Value.kForward);
          motors.set_climb_task("7-extend climber(3rd bar)");
        }
        break;
      case "7-extend climber(3rd bar)":
        motors.climber_down();
        if(motors.get_encoders() > 2000000) {
          motors.climber_stop();
          motors.set_climb_task("8-retract climber(3rd bar)");
        }
        break;
      case "8-retract climber(3rd bar)":
        pneumatics.dblSolenoid(DoubleSolenoid.Value.kReverse);
        motors.climber_up();
        if(motors.get_encoders() < 10000 ) {
          motors.climber_stop();
          motors.set_climb_task("9-tilt climber(4th bar)");
        }
        break;
      case "9-tilt climber(4th bar)":
        motors.climber_down();
        if(motors.get_encoders() > 500000) {
          motors.climber_stop();
          pneumatics.dblSolenoid(DoubleSolenoid.Value.kForward);
          motors.set_climb_task("10-extend climber(4th bar)");
        }
        break;
      case "10-extend climber(4th bar)":
        motors.climber_down();
        if(motors.get_encoders() > 2000000) {
          motors.climber_stop();
          motors.set_climb_task("11-retract climber(4th bar)");
        }
        break;
      case "11-retract climber(4th bar)":
        pneumatics.dblSolenoid(DoubleSolenoid.Value.kReverse);
        motors.climber_up();
        if(motors.get_encoders() < 10000) {
          motors.climber_stop();
        }
        break;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
