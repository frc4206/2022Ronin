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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.VisionAlignStopCommand;
import frc.robot.commands.conveyor.ConveyorAutoCommand;
import frc.robot.commands.conveyor.ConveyorForwardCommand;
import frc.robot.commands.harvestor.HarvestorOutCommand;
import frc.robot.commands.shooter.ShooterWallHubCommand;
import frc.robot.commands.shooter.ShooterWallHubPlusCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Kevins3to5BallOriganalAuto extends SequentialCommandGroup {
  public Kevins3to5BallOriganalAuto(SwerveSubsystem s_Swerve, HarvestorSubsystem m_harvestor, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter, PneumaticsSubsystem m_pneumatics){
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecondfast,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredfast)
            .setKinematics(Constants.Swerve.swerveKinematics);


    //------------An example trajectory to follow.  All units in meters.-------------------//
    Trajectory tarjectoryPart1 =
        TrajectoryGenerator.generateTrajectory(
            // start facng the left
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),

            //points to grab second cargo
            List.of(new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(5)), 

                    new Translation2d(Units.inchesToMeters(-20), Units.inchesToMeters(5))),

            //spin around and grab second cargp
            new Pose2d(Units.inchesToMeters(-40), Units.inchesToMeters(5), new Rotation2d(Units.degreesToRadians(170))),
            config);

    Trajectory tarjectoryPart2 =
        TrajectoryGenerator.generateTrajectory(
        //go to pickup the second ball you need for the auto
        new Pose2d(Units.inchesToMeters(-40), Units.inchesToMeters(5), new Rotation2d(Units.degreesToRadians(-170))),
            List.of(
                new Translation2d(Units.inchesToMeters(-20), Units.inchesToMeters(25)),

                new Translation2d(Units.inchesToMeters(-20), Units.inchesToMeters(50)),


                new Translation2d(Units.inchesToMeters(-0), Units.inchesToMeters(75))

        ),
        new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(100), new Rotation2d(Units.degreesToRadians(90))),
        config);

    Trajectory tarjectoryPart3 =
        TrajectoryGenerator.generateTrajectory(
            //Turn and move back to face the hub so you can shoot the cargo
            new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(100), new Rotation2d(Units.degreesToRadians(120))),
            List.of(new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(70)), 
    
                    //new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(55)),
                
                    new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(25))),

                
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(10), new Rotation2d(Units.degreesToRadians(0))),
            config);



    Trajectory tarjectoryPart4 =
        TrajectoryGenerator.generateTrajectory(
            // Sprint to the terminal to do an 4-5 ball
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(10), new Rotation2d(Units.degreesToRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(50)), 
                new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(70)), 
                //new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(15)),
                //new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(15)), 
                new Translation2d(Units.inchesToMeters(-20), Units.inchesToMeters(170))),
                    
                    
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-25), Units.inchesToMeters(235), new Rotation2d(Units.degreesToRadians(100))),
            config);



            Trajectory tarjectoryPart5 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.inchesToMeters(-25), Units.inchesToMeters(235), new Rotation2d(Units.degreesToRadians(100))),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                  new Translation2d(Units.inchesToMeters(-25), Units.inchesToMeters(170)),
            
                  new Translation2d(Units.inchesToMeters(-25), Units.inchesToMeters(120)), 

                  new Translation2d(Units.inchesToMeters(-25), Units.inchesToMeters(40))), 


                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(Units.inchesToMeters(-25), Units.inchesToMeters(23), new Rotation2d(Units.degreesToRadians(0))),
                config);

            


    //------------------------The PID Controller for the actual auto------------------------//
    var thetaController =
        new ProfiledPIDController(
            5, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
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

        new ParallelCommandGroup(
          new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(1.2),
          new ShooterWallHubPlusCommand(m_shooter).withTimeout(1.2)

        ),

        new ParallelCommandGroup(
            new ShooterWallHubPlusCommand(m_shooter).withTimeout(0.2),
            new ConveyorForwardCommand(m_conveyor).withTimeout(0.2),
            new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(0.2)
        ),

        //go to first ball
         new ParallelCommandGroup(
            drivingPart1,
             new HarvestorOutCommand(m_harvestor, m_pneumatics).withTimeout(0.5),
             new ShooterWallHubPlusCommand(m_shooter).withTimeout(0.7)

         ),

        //go back near ball 2 while spinning

            new ParallelCommandGroup(
                 drivingPart2,
                new ShooterWallHubPlusCommand(m_shooter).withTimeout(1.5),
                new ConveyorForwardCommand(m_conveyor).withTimeout(0.5)

           ),


        // go back to the hub and ready to shoot
        new ParallelCommandGroup(
             drivingPart3,
          new ShooterWallHubPlusCommand(m_shooter).withTimeout(1),
          new ConveyorAutoCommand(m_conveyor)

         ),

        //align to shoot again

         new ParallelCommandGroup(
             new ShooterWallHubPlusCommand(m_shooter).withTimeout(1.2),
             new ConveyorForwardCommand(m_conveyor).withTimeout(1.2),
             new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(1.2),
             new HarvestorOutCommand(m_harvestor, m_pneumatics).withTimeout(0.1)

         ),

        //go to terminal for last 2 cargo
        new ParallelCommandGroup(
           drivingPart4,
          new ShooterWallHubCommand(m_shooter).withTimeout(3),
          new HarvestorOutCommand(m_harvestor, m_pneumatics).withTimeout(0.1)
        ),

        //wait a tiny second 
        // new ParallelCommandGroup(
        //     new ConveyorAutoCommand(m_conveyor).withTimeout(0.2),
        //     new HarvestorOutCommand(m_harvestor, m_pneumatics).withTimeout(0.2)
        // ),
            

        //drive back to the hub 
         new ParallelCommandGroup(
             drivingPart5,
             new ConveyorAutoCommand(m_conveyor).withTimeout(4),
             new ShooterWallHubCommand(m_shooter).withTimeout(3)
         ),


         //aim at the hub
         new ParallelCommandGroup(
            new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(0.2),
            new ShooterWallHubPlusCommand(m_shooter).withTimeout(0.2)
         ),
         //shoot the last 2 cargo
         new ParallelCommandGroup(
             new ShooterWallHubCommand(m_shooter).withTimeout(1),
             new ConveyorForwardCommand(m_conveyor).withTimeout(1),
             new VisionAlignStopCommand(s_Swerve, true, true).withTimeout(1)
         )


    );
}
}
