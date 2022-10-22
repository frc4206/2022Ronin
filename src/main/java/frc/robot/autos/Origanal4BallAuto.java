// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Origanal4BallAuto extends SequentialCommandGroup {
  public Origanal4BallAuto(SwerveSubsystem s_Swerve, HarvestorSubsystem m_harvestor, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter, PneumaticsSubsystem m_pneumatics){
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);


    //------------An example trajectory to follow.  All units in meters.-------------------//
    Trajectory tarjectoryPart1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, -Units.inchesToMeters(15)), 

                    new Translation2d(0, -Units.inchesToMeters(30))),

            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, Units.inchesToMeters(-40), new Rotation2d(Units.degreesToRadians(-90))),
            config);

    Trajectory tarjectoryPart2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, Units.inchesToMeters(-40), new Rotation2d(Units.degreesToRadians(-90))),
            List.of(
                new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(-30)),

                new Translation2d(Units.inchesToMeters(-6), Units.inchesToMeters(-15)),


                new Translation2d(Units.inchesToMeters(6), Units.inchesToMeters(-0))

        ),
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(10),  Rotation2d.fromDegrees(70)), config);

    Trajectory tarjectoryPart3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(10), new Rotation2d(Units.degreesToRadians(70))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(Units.inchesToMeters(-30), Units.inchesToMeters(-5)), 
    
                    new Translation2d(Units.inchesToMeters(-60), Units.inchesToMeters(-10)),
                
                    new Translation2d(Units.inchesToMeters(-80), Units.inchesToMeters(-10))),

                
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-90), Units.inchesToMeters(-50), new Rotation2d(Units.degreesToRadians(180))),
            config);



    Trajectory tarjectoryPart4 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(Units.inchesToMeters(-90), Units.inchesToMeters(-50), new Rotation2d(Units.degreesToRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(Units.inchesToMeters(-130), Units.inchesToMeters(-40)), 
                new Translation2d(Units.inchesToMeters(-180), Units.inchesToMeters(-20)), 

                    new Translation2d(Units.inchesToMeters(-200), Units.inchesToMeters(-15))),
                    
                    
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-220), Units.inchesToMeters(-15), new Rotation2d(Units.degreesToRadians(180))),
            config);



            Trajectory tarjectoryPart5 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.inchesToMeters(-220), Units.inchesToMeters(-15), new Rotation2d(Units.degreesToRadians(180))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                    new Translation2d(Units.inchesToMeters(-180), Units.inchesToMeters(-10)),
            
                    new Translation2d(Units.inchesToMeters(-140), Units.inchesToMeters(0)),

                    new Translation2d(Units.inchesToMeters(-80), Units.inchesToMeters(0)),

                    new Translation2d(Units.inchesToMeters(-40), Units.inchesToMeters(10))),                         
                        
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(20), new Rotation2d(Units.degreesToRadians(70))),
                config);

            


    //------------------------The PID Controller for the actual auto------------------------//
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    //------------------------Making Of driving Commands----------------------------------//
    SwerveControllerCommand drivingPart1 =
        new SwerveControllerCommand(
            tarjectoryPart1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXControllerfast, 0, 0),
            new PIDController(Constants.AutoConstants.kPYControllerfast, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);  
            
    SwerveControllerCommand drivingPart2 =
        new SwerveControllerCommand(
              tarjectoryPart2,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXControllerfast, 0, 0),
              new PIDController(Constants.AutoConstants.kPYControllerfast, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);   
              
    SwerveControllerCommand drivingPart3 =
        new SwerveControllerCommand(
              tarjectoryPart3,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXControllerfast, 0, 0),
              new PIDController(Constants.AutoConstants.kPYControllerfast, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);  

    SwerveControllerCommand drivingPart4 =
        new SwerveControllerCommand(
              tarjectoryPart4,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(Constants.AutoConstants.kPXControllerfast, 0, 0),
              new PIDController(Constants.AutoConstants.kPYControllerfast, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);  

              SwerveControllerCommand drivingPart5 =
              new SwerveControllerCommand(
                    tarjectoryPart5,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXControllerfast, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYControllerfast, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);  


    //---------------------The Actual Command List That will Run-----------------//

    addCommands(

        //resets odemetry
        new InstantCommand(() -> s_Swerve.resetOdometry(tarjectoryPart1.getInitialPose())),

        //go to first ball
        // new ParallelCommandGroup(
            drivingPart1,
        //     new HarvestorOutCommand(m_harvestor, m_pneumatics).withTimeout(0.5)
        // ),

        // //go back near ball 2 and spin
        // new ParallelCommandGroup(

            //new ParallelRaceGroup(
                drivingPart2,
        //        new ShooterXSpotCommand(m_shooter).withTimeout(0.5),
        //     //),
            
        //     new ConveyorForwardCommand(m_conveyor).withTimeout(0.5)
        // ),
        
        // //align to shoot 
        
        // new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(0.5),

        // new ParallelCommandGroup(
        //     new ShooterXSpotCommand(m_shooter).withTimeout(1.5),
        //     new ConveyorForwardCommand(m_conveyor).withTimeout(1.5),
        //     new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(1.5)
        // ),

        //pickup ball three
        //new ParallelCommandGroup(
            drivingPart3,
        //     new HarvestorOutCommand(m_harvestor, m_pneumatics).withTimeout(0.5)

        // ),

        // //align to shoot again
        // new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(0.5),

        // new ParallelCommandGroup(
        //     new ShooterXSpotCommand(m_shooter).withTimeout(1.5),
        //     new ConveyorForwardCommand(m_conveyor).withTimeout(1.5),
        //     new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(1.5)
        // ),

        //go to terminal
        drivingPart4,

        // new ParallelCommandGroup(
            drivingPart5//,
        //     new ConveyorForwardCommand(m_conveyor).withTimeout(0.2),
        //     new ShooterWallHubCommand(m_shooter)
        // ),

        // new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(0.5),

        // new ParallelCommandGroup(
        //     new ShooterWallHubCommand(m_shooter).withTimeout(1.5),
        //     new ConveyorForwardCommand(m_conveyor).withTimeout(1.5),
        //     new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(1.5)
        // )
        

        //zeros gyro
        //new InstantCommand(() -> s_Swerve.zeroGyro()).withTimeout(0.1)

    );
}
}