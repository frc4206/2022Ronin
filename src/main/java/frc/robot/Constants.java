package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static String Canivore1 = "Canivore1";
    
    public final class MotorsIDs {
        //kellie
        public static final int climberfalconMotorR = 25;
        public static final int climberfalconMotorL = 24;

        //kellie
        public static final int shooterUpperMotor = 22;
        public static final int shooterLowerMotor = 21;

        //reggie
        public static final int groundmotor = 27; 

        //andrew 
        public static final int conveyormotor = 29; 

    }

    public final class Pneumatics{
        //kellie and Mr Blair
        public static final int climberSolenoidFWD = 4;
        public static final int climberSolenoidBKWD = 5;
        public static final int pneumaticPressureSensor = 0;

        //reggie
        public static final int harvestorSolenoidFWD = 2;
        public static final int harvestorSolenoidBKWD = 3; 

        //andrew
        public static final int pankakeSolonoidFWD = 7;
        public static final int pankakeSolonoidBKWD = 6;

    }

    public final class MotorValues{
        //reggie
        public static final double FeederIn = -1;
        public static final double FeederOut = 1;
        public static final double  FeederStop = 0;

        //andrew
        public static final double conveyorForward = 1;
        public static final double conveyorBackward = -0.5;
        public static final double  conveyorStop = 0;


        public static final double shooteruppermanual = 0.5;
        public static final double  shooterlowermanual = 0.5;
    }

    public static final class Swerve {
        public static final int pigeonID = 30;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(23.75);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);//change to 3.7ish for MK4s when sure 
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final double angleOffset = 199.69;//314.5 these aren't accurate just refences
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 9;
            public static final double angleOffset = 172.88;//246.7
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }



        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final double angleOffset = 314.03;//264.4
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 11;
            public static final double angleOffset = 125.68;//226.4
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxSpeedMetersPerSecondfast = 5;
        public static final double kMaxAccelerationMetersPerSecondSquaredfast = 5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPXControllerfast = 9;
        public static final double kPYControllerfast = 9;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
