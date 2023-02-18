// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;


/** Add your docs here. */
public class Constants {

    public static final class Controllers {
        public static final int kDriverControllerPort = 0; 
        public static final int kManipulatorControllerPort = 1; 
        public static final double kDeadband = 0.1; 
    }

    public static final class Arm {
        /**
         * The CAN ID of the extension motor.
         */
        public static final int kExtensionMotorId = 21; // TODO: Fix

        /*
         * The CAN ID of the elevation motor.
         */
        public static final int kTiltMotorId = 20; // TODO: Fix

        /*
         * The gear ratio for the arm that converts motor rotations into inches
         * of extension of the arm.
         */
        public static final double kExtendRotationsToInches = (1.0 / 2.0) / 4.0;

        /*
         * The gear ratio for the arm that converts motor rotations into inches
         * of elevation of the arm.
         */
        public static final double kTiltRotationsToInches = (1.0 / 2.0) / 3.0; // This is not completely accurate so it's been changed to rotations
        /*
         * The maximum distance (in inches) that the arm can extend.
         */
        public static final double kExtendForwardLimit = 30; 

         /*
         * The maximum distance (in inches) that the arm can retract.
         */
        public static final double kExtendReverseLimit = 0.0; 

         /*
         * The maximum distance (in inches) that the arm can elevate.
         */
        public static final double kTiltForwardLimit = 18; 

         /*
         * The maximum distance (in rotations) that the arm can go down.
         */
        public static final double kTiltReverseLimit = 0.0;

        public static final double kMinSafeTilt = 2.6;

        
        /*
         * Tilt PID Controller
         */
        public static final double kTiltP = 3;
        /*
         * Tilt Min Speed
         */
        public static final double kTiltMinOutput = -1;
        /*
         * Tilit Max Speed
         */
        public static final double kTiltMaxOutput = 1;

        /*
         * Extension PID Controller
         */
        public static final double kExtensionP = 3;
        /*
         * Extension Min Speed
         */
        public static final double kExtensionMinOutput = -1;
        /*
         * Extension Max Speed
         */
        public static final double kExtensionMaxOutput = 1;


        /*
         * High Cone Tilt Height
         */
        public static final double kHighConeTilt = 1;

        /*
         * Middle Cone Tilt Height
         */
        public static final double kMiddleConeTilt = 1;

        /*
         * Low Cone Tilt Height
         */
        public static final double kLowConeTilt = 1;

        /*
         * High Cone Extension
         */
        public static final double kHighConeExtend = 1;

        /*
         * Middle Cone Extension
         */
        public static final double kMiddleConeExtend = 1;

        /*
         * Low Cone Extension
         */
        public static final double kLowConeExtend = 1;

        

        /*
         * High Cube Tilt Height
         */
        public static final double kHighCubeTilt = 1;

        /*
         * Middle Cube Tilt Height
         */
        public static final double kMiddleCubeTilt = 1;

        /*
         * Low Cube Tilt Height
         */
        public static final double kLowCubeTilt = 1;

        /*
         * High Cube Extension
         */
        public static final double kHighCubeExtend = 1;

        /*
         * Middle Cube Extension
         */
        public static final double kMiddleCubeExtend = 1;

        /*
         * Low Cube Extension
         */
        public static final double kLowCubeExtend = 1;


    }

    public static final class Clamps {

    }

    public static final class Drive {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(18.75);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(21.25);
        // Distance between front and rear wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = 5.9215;
        public static final double kFrontRightChassisAngularOffset = 2.85;
        public static final double kRearLeftChassisAngularOffset = 1.60;
        public static final double kRearRightChassisAngularOffset = 4.45;

        /*   public static final double kFrontLeftChassisAngularOffset = 5.9175;
        public static final double kFrontRightChassisAngularOffset = 2.8580;
        public static final double kRearLeftChassisAngularOffset = 1.5195;
        public static final double kRearRightChassisAngularOffset = 4.4545; */
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 15;
        public static final int kRearLeftDrivingCanId = 7;
        public static final int kFrontRightDrivingCanId = 16;
        public static final int kRearRightDrivingCanId = 9;
    
        public static final int kFrontLeftTurningCanId = 14;
        public static final int kRearLeftTurningCanId = 6;
        public static final int kFrontRightTurningCanId = 17;
        public static final int kRearRightTurningCanId = 8;
    
        public static final boolean kGyroReversed = false;
      }
    
      public static final class SwerveModule {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kNeoMotorFreeSpeedRpm = 5676 * 0.9;

        public static final double kDrivingMotorFreeSpeedRps = kNeoMotorFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.1016; // 3 : 0.0762; // 3.75 : 0.09525; //3.8 : 0.09652; // 4 : 0.1016; //Units.inchesToMeters(4.0);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (50.0 * 17.0 * 45.0) / (kDrivingMotorPinionTeeth * 27.0 * 15.0);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    
        public static final double kSteeringMotorReduction = 150.0 / 7.0;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kSteeringMotorReduction; // radians
        public static final double kTurningEncoderVelocityFactor = ((2 * Math.PI) / kSteeringMotorReduction) / 60.0; // radians per second

        public static final double kTurningAnalogPositionFactor = (Math.PI * 2 / 3.3);
        public static final double kTurningAnalogVelocityFactor = (kTurningAnalogPositionFactor/60);
    
        public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians
    
        public static final double kDrivingP = 0.1;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0.01;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 2.295;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0.2295;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
      }

      public static final class Intake {

        public static final int kIntakeRollersCANId = 19;

        public static final int kIntakeArmCANId = 18;

        public static final double kExtendSpeed = 0.3; 
        public static final double kRetractSpeed = -0.3; 

        public static final double kRollersInward = 0.5;
        public static final double kRollersOutward = -0.5;

        public static final int kCurrentLimit = 30;

        /**
         * The color of the cube, as detected by the REV Color Sensor V3.
         */
        public static final Color kCubeColor = new Color(0, 0, 0); // TODO: Update Color values

        /**
         * The color of the cone, as detected by the REV Color Sensor V3.
         */
        public static final Color kConeColor = new Color(0, 0, 0); // TODO: Update Color values

        /**
         * The minimum distance to the cargo in order to consider it to be present,
         * as detected by the REV Color Sensor V3.
         */
        public static final int kDistance = 200; // TODO: Update 
      }

      public static final class PrettyLights {

      }

    public static final class Suction {
        /**
         * The CAN ID of the first suction motor.
         */
        public static final int kMotorBottomId = 10;

        /**
         * The CAN ID of the second suction motor.
         */
        public static final int kMotorTopId = 11;

        /**
         * The pneumatic hub channel ID of the first suction solenoid.
         */
        public static final int kSolenoidBottomId = 0;

        /**
         * The pneumatic hub channel ID of the second suction solenoid.
         */
        public static final int kSolenoidTopId = 1;

        /**
         * The maximum current to send to the suction motor.
         */
        public static final int kCurrentLimit = 30;

        /**
         * The maximum speed that the suction motor runs during regular hold of the game piece.
         */
        public static final double kMaxSpeed = 0.33;

        /*
         * Delay for Disabling sequence
         */
        public static final double kDelay = 0.5;

        /**
         * The pneumatic hub channel ID of the bottom pressure sensor.
         */
        public static final int kPressureSensorBottomId = 0;

        /**
         * The pneumatic hub channel ID of the top pressure sensor.
         */
        public static final int kPressureSensorTopId = 1;

        /*
         * The target pressure for bottom vacuum state in PSI
         */
        public static final double kTargetPressureBottom = 17.5;

        /*
         * The minimum pressure for bottom vacuum state in PSI
         */
        public static final double kMinimumPressureBottom = 19.5;

        /*
         * The target pressure for top vacuum state in PSI
         */
        public static final double kTargetPressureTop = 17.5;

        /*
         * The minimum pressure for top vacuum state in PSI
         */
        public static final double kMinimumPressureTop = 19.5;

      }

      public static final class Vision {
        public static AprilTagFieldLayout kAprilTagFieldLayout = null;
        static {
            try {
                kAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public static final String kLeftCameraName = "LEFT";
        public static final Transform3d kLeftRobotToCamera =
            new Transform3d(
                new Translation3d(-0.16390, 0.18440, 0.59055),
                new Rotation3d(0, 0, Math.toRadians(10))); 

        public static final String kRightCameraName = "RIGHT";
        public static final Transform3d kRightRobotToCamera =
            new Transform3d(
                new Translation3d(-0.16390, -0.18298, 0.59055),
                new Rotation3d(0, 0, Math.toRadians(10))); 

      }

}
