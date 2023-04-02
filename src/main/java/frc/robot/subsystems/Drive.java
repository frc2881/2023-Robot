// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.lib.NavX;
import frc.robot.lib.Node;
import frc.robot.lib.PhotonCameraWrapper;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.Utils;
import frc.robot.lib.Node.NodeType;

public class Drive extends SubsystemBase {

  public static enum SwerveDriveMode {
    FIELD_CENTRIC, 
    ROBOT_CENTRIC
  }

  private final SwerveModule m_frontLeft = new SwerveModule(
    SwerveModule.Location.FrontLeft,
    Constants.Drive.kFrontLeftDrivingCanId,
    Constants.Drive.kFrontLeftTurningCanId,
    Constants.Drive.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    SwerveModule.Location.FrontRight,
    Constants.Drive.kFrontRightDrivingCanId,
    Constants.Drive.kFrontRightTurningCanId,
    Constants.Drive.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
    SwerveModule.Location.RearLeft,
    Constants.Drive.kRearLeftDrivingCanId,
    Constants.Drive.kRearLeftTurningCanId,
    Constants.Drive.kRearLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
    SwerveModule.Location.RearRight,
    Constants.Drive.kRearRightDrivingCanId,
    Constants.Drive.kRearRightTurningCanId, 
    Constants.Drive.kRearRightChassisAngularOffset);

  private final NavX m_gyro = new NavX();
  
  private SwerveDriveMode m_swerveDriveMode = SwerveDriveMode.FIELD_CENTRIC;

  private boolean m_isXConfiguration = false;

  public PhotonCameraWrapper m_leftPhotonCamera;
  public PhotonCameraWrapper m_rightPhotonCamera;

  private final DoubleLogEntry m_logRoll;
  private final DoubleLogEntry m_logYaw;
  private final DoubleLogEntry m_logPitch;

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

  private List<Node> m_nodes = null;
  List<Pose2d> m_nodePoses = null;

  public Drive() {
    DataLog log = DataLogManager.getLog();
    m_logRoll = new DoubleLogEntry(log, "/Drive/roll");
    m_logYaw = new DoubleLogEntry(log, "/Drive/yaw");
    m_logPitch = new DoubleLogEntry(log, "/Drive/pitch");
  }

  @Override
  public void periodic() {
    updateNodes();
    updatePhotonCameras();
    updatePose();
    updateTelemetry();
    sampleModules();
  }
  
  private void updatePhotonCameras() {
    if ( m_leftPhotonCamera != null &&  m_rightPhotonCamera != null) { return; }

    Alliance alliance = DriverStation.getAlliance();

    if (alliance != Alliance.Invalid) {
      Constants.Vision.kAprilTagFieldLayout.setOrigin(
        alliance == Alliance.Red 
          ? OriginPosition.kRedAllianceWallRightSide
          : OriginPosition.kBlueAllianceWallRightSide);

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
    }
  }

  public void resetPhotonCameras() {
    if (m_leftPhotonCamera != null) {
      m_leftPhotonCamera.dispose();
      m_leftPhotonCamera = null;
    }
    if (m_rightPhotonCamera != null) {
      m_rightPhotonCamera.dispose();
      m_rightPhotonCamera = null;
    }
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
    if (!RobotState.isAutonomous()) {
      if (!updateVisionMeasurement(m_leftPhotonCamera)) {
        updateVisionMeasurement(m_rightPhotonCamera);
      }  
    }     
  }

  private boolean updateVisionMeasurement(PhotonCameraWrapper photonCamera) {
    Pose2d estimatedPose = new Pose2d();
    if (photonCamera != null) {
      Optional<EstimatedRobotPose> pipelineResult = photonCamera.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
      if (pipelineResult.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = pipelineResult.get();
        estimatedPose = estimatedRobotPose.estimatedPose.toPose2d();
        if (Utils.isPoseInBounds(estimatedPose, Constants.Vision.kFieldMinPose, Constants.Vision.kFieldMaxPose)) {
          m_poseEstimator.addVisionMeasurement(estimatedPose, estimatedRobotPose.timestampSeconds);
          return true;
        } 
      }
    }
    return false;
  }

  /**
   * Returns the current estimated pose of the robot.
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

  public Node getNearestNode() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    Pose2d nearestNodePose = currentPose.nearest(m_nodePoses);

    return m_nodes
      .stream()
      .filter(node -> nearestNodePose.equals(node.pose))
      .findFirst()
      .orElse(Constants.Vision.kDefaultNode);
  }

  public NodeType getNearesNodeType() {
    return getNearestNode().nodeType;
  }

  private void updateNodes() {
    if (m_nodes != null) { return; }

    Alliance alliance = DriverStation.getAlliance();

    if (alliance != Alliance.Invalid) {
      m_nodes = alliance == Alliance.Red ? 
        Constants.Vision.kNodesRedAlliance :
        Constants.Vision.kNodesBlueAlliance;
      m_nodePoses = new ArrayList<Pose2d>();
      m_nodes.forEach((node) -> m_nodePoses.add(node.pose));
    }
  }

  public PathPlannerTrajectory getTrajectoryForNearestNode() {
    Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
    Pose2d nearestNodePose = getNearestNode().pose;

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
      new PathConstraints(0.5, 0.5), 
      new PathPoint(
        currentPose.getTranslation(), 
        Rotation2d.fromDegrees(180), 
        m_gyro.getRotation2d()
      ),
      new PathPoint(
        nearestNodePose.getTranslation(), 
        Rotation2d.fromDegrees(180), 
        nearestNodePose.getRotation()
      )
    );

    return PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
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
    if(!m_isXConfiguration){
      var swerveModuleStates = Constants.Drive.kDriveKinematics.toSwerveModuleStates(
        (m_swerveDriveMode == SwerveDriveMode.FIELD_CENTRIC)
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_gyro.getAngle()))
          : new ChassisSpeeds(xSpeed, ySpeed, rot));

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.kMaxSpeedMetersPerSecond);

      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  public void setDriveMode(SwerveDriveMode driveMode) {
    m_swerveDriveMode = driveMode;
    SmartDashboard.putString("Drive/Swerve/Mode", m_swerveDriveMode.toString());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setXConfiguration() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void toggleX() {
    // if u push the button, x goes to true and pushed is set to false
    // if u push the button again, x goes to false, pushed goes to true and setX ends
    if (!m_isXConfiguration) {
      setXConfiguration();
    }
    m_isXConfiguration = !m_isXConfiguration;
    SmartDashboard.putBoolean("Drive/Swerve/IsXConfiguration", m_isXConfiguration);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    if (!m_isXConfiguration) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drive.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
    }
  }

  private void sampleModules(){
    m_frontLeft.sample();
    m_frontRight.sample();
    m_rearLeft.sample();
    m_rearRight.sample();
  }
   
  public void resetSwerve() {
    m_frontLeft.resetTurningEncoder();
    m_frontRight.resetTurningEncoder();
    m_rearLeft.resetTurningEncoder();
    m_rearRight.resetTurningEncoder();
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

  public void zeroHeadingToAng(double angle) {
    m_gyro.resetToAng(angle);
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

  public double getRoll() {
    return m_gyro.getRoll();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    m_frontLeft.initSendable(builder);
    m_frontRight.initSendable(builder);
    m_rearLeft.initSendable(builder);
    m_rearRight.initSendable(builder);

    m_gyro.initSendable(builder);
    builder.addDoubleProperty("Heading", this::getHeading, null);
    builder.addDoubleProperty("TurnRate", this::getTurnRate, null);
  }

  private void updateTelemetry() {
    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumberArray("Drive/Pose",  new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
    SmartDashboard.putNumber("Drive/Roll", m_gyro.getRoll());
  }

  public void logDrive() {
    m_logRoll.append(m_gyro.getRoll());
    m_logYaw.append(m_gyro.getYaw());
    m_logPitch.append(m_gyro.getPitch());
  }
}
