// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.NavX;
import frc.robot.lib.PhotonCameraWrapper;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.Enums.SwerveDriveMode;

public class Drive extends SubsystemBase {
  // Create SwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      Constants.Drive.kFrontLeftDrivingCanId,
      Constants.Drive.kFrontLeftTurningCanId,
      Constants.Drive.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new  SwerveModule(
      Constants.Drive.kFrontRightDrivingCanId,
      Constants.Drive.kFrontRightTurningCanId,
      Constants.Drive.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new  SwerveModule(
      Constants.Drive.kRearLeftDrivingCanId,
      Constants.Drive.kRearLeftTurningCanId,
      Constants.Drive.kRearLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new  SwerveModule(
      Constants.Drive.kRearRightDrivingCanId,
      Constants.Drive.kRearRightTurningCanId, 
      Constants.Drive.kRearRightChassisAngularOffset);

  // The gyro sensor
  private final NavX m_gyro = new NavX();
  
  private SwerveDriveMode m_driveMode = SwerveDriveMode.FIELD_CENTRIC;

  public PhotonCameraWrapper m_leftPhotonCamera;
  public PhotonCameraWrapper m_rightPhotonCamera;

  private final SwerveDrivePoseEstimator m_poseEstimator = 
    new SwerveDrivePoseEstimator(
      Constants.Drive.kDriveKinematics, 
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()}, 
      new Pose2d());

  private final Field2d m_field = new Field2d();

  public Drive() {
    m_leftPhotonCamera = new PhotonCameraWrapper(
      Constants.Vision.kLeftCameraName,
      Constants.Vision.kLeftRobotToCamera,
      PoseStrategy.MULTI_TAG_PNP,
      Constants.Vision.kAprilTagFieldLayout
    );
    m_rightPhotonCamera = new PhotonCameraWrapper(
      Constants.Vision.kRightCameraName,
      Constants.Vision.kRightRobotToCamera,
      PoseStrategy.MULTI_TAG_PNP,
      Constants.Vision.kAprilTagFieldLayout
    );

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    updatePose();
    updateTelemetry();
  }
   
  public void updatePose() {
    m_poseEstimator.update(
      m_gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    });

    Optional<EstimatedRobotPose> leftCameraResult = m_leftPhotonCamera.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
    if (leftCameraResult.isPresent()) {
      EstimatedRobotPose camPose = leftCameraResult.get();
      m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    } else {
      Optional<EstimatedRobotPose> rightCameraResult = m_rightPhotonCamera.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
      if (rightCameraResult.isPresent()) {
        EstimatedRobotPose camPose = rightCameraResult.get();
        m_poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      }
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    // Adjust input based on   speed
    // xSpeed *= Constants.Drive.kMaxSpeedMetersPerSecond;
    // ySpeed *= Constants.Drive.kMaxSpeedMetersPerSecond;
    // rot *= Constants.Drive.kMaxAngularSpeed;

    var swerveModuleStates = Constants.Drive.kDriveKinematics.toSwerveModuleStates(
        (m_driveMode == SwerveDriveMode.FIELD_CENTRIC)
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Drive.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setDriveMode(SwerveDriveMode driveMode) {
    m_driveMode = driveMode;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.Drive.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.Drive.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    m_gyro.initSendable(builder);
  }

  private void updateTelemetry() {
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putString("Drive/Mode", m_driveMode.toString());
    SmartDashboard.putNumber("Drive/Heading", getHeading());
    SmartDashboard.putNumber("Drive/TurnRate", getTurnRate());

    SmartDashboard.putNumber("Drive/Swerve/FrontLeft/Steering/RelativePosition", m_frontLeft.getSteeringRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/FrontLeft/Steering/AbsolutePosition", m_frontLeft.getSteeringAbsolutePosition());
    SmartDashboard.putNumber("Drive/Swerve/FrontLeft/Driving/RelativePosition", m_frontLeft.getDrivingRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/FrontLeft/Driving/Velocity", m_frontLeft.getDrivingVelocity());

    SmartDashboard.putNumber("Drive/Swerve/FrontRight/Steering/RelativePosition", m_frontRight.getSteeringRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/FrontRight/Steering/AbsolutePosition", m_frontRight.getSteeringAbsolutePosition());
    SmartDashboard.putNumber("Drive/Swerve/FrontRight/Driving/RelativePosition", m_frontRight.getDrivingRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/FrontRight/Driving/Velocity", m_frontRight.getDrivingVelocity());

    SmartDashboard.putNumber("Drive/Swerve/RearLeft/Steering/RelativePosition", m_rearLeft.getSteeringRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/RearLeft/Steering/AbsolutePosition", m_rearLeft.getSteeringAbsolutePosition());
    SmartDashboard.putNumber("Drive/Swerve/RearLeft/Driving/RelativePosition", m_rearLeft.getDrivingRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/RearLeft/Driving/Velocity", m_rearLeft.getDrivingVelocity());

    SmartDashboard.putNumber("Drive/Swerve/RearRight/Steering/RelativePosition", m_rearRight.getSteeringRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/RearRight/Steering/AbsolutePosition", m_rearRight.getSteeringAbsolutePosition());
    SmartDashboard.putNumber("Drive/Swerve/RearRight/Driving/RelativePosition", m_rearRight.getDrivingRelativePosition());
    SmartDashboard.putNumber("Drive/Swerve/RearRight/Driving/Velocity", m_rearRight.getDrivingVelocity());

    SmartDashboard.putNumber("Drive/Gyro/Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Drive/Gyro/Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("Drive/Gyro/Pitch", m_gyro.getPitch());
  }
}
