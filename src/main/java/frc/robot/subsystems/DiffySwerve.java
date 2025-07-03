// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMap.PodConfig;
import frc.robot.Robot;

public class DiffySwerve extends SubsystemBase {
	private final Pigeon2 gyro;

	private Pigeon2SimState gyroSimState;

	private final ArrayList<DrivePod> pods = new ArrayList<DrivePod>();

	private final SwerveDrivePoseEstimator poseEstimator;

	public double targetAngle = 0;

	private final Field2d field2d = new Field2d();

	StructArrayPublisher<SwerveModuleState> SMSPublisher;
	StructPublisher<Pose2d> PosePublisher;
	StructPublisher<ChassisSpeeds> ChassisSpeedsPublisher;
	DoubleEntry azimuthkPSub;
	DoubleEntry azimuthkISub;
	DoubleEntry azimuthkDSub;

	double simGyroPosition = 0;

	public DiffySwerve() {
		gyro = new Pigeon2(RobotMap.pigeonID);

		for (int i = 0; i < RobotMap.PodConfigs.length; i++) {
			pods.add(new DrivePod(i, RobotMap.PodConfigs[i].encoderID, RobotMap.PodConfigs[i].leftMotorID,
					RobotMap.PodConfigs[i].rightMotorID, PodConfig.leftMotorInvert, PodConfig.rightMotorInvert,
					RobotMap.PodConfigs[i].encoderOffset, PodConfig.encoderInvert, PodConfig.ampLimit,
					PodConfig.motorsBrake, PodConfig.rampRate, PodConfig.kP, PodConfig.kI, PodConfig.kD, PodConfig.motorGearing));
		}

		// initialze suppliers for information-sharing between pods
		initializeSuppliers();

		// initialize pod positions
		SwerveModulePosition positions[] = pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);

		// initialize odometry based on the pod positions
		poseEstimator = new SwerveDrivePoseEstimator(
				RobotMap.drivetrainKinematics,
				getGyro(),
				getModulePositions(),
				new Pose2d());

		configurePathPlanner();

		// gyro.configAllSettings(new Pigeon2Configuration());
		//TODO: make pigeon config if necessary

		// initialize telemetry publishers
		SMSPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
				.publish();
		PosePublisher = NetworkTableInstance.getDefault().getStructTopic("RobotPose", Pose2d.struct).publish();
		ChassisSpeedsPublisher = NetworkTableInstance.getDefault().getStructTopic("ChassisSpeeds", ChassisSpeeds.struct)
				.publish();

		// initialize PID subscribers
		if(Constants.tuningMode) {
			azimuthkPSub = NetworkTableInstance.getDefault().getTable("PIDs").getDoubleTopic("kP").getEntry(PodConfig.kP);
			azimuthkISub = NetworkTableInstance.getDefault().getTable("PIDs").getDoubleTopic("kI").getEntry(PodConfig.kI);
			azimuthkDSub = NetworkTableInstance.getDefault().getTable("PIDs").getDoubleTopic("kD").getEntry(PodConfig.kD);

			azimuthkPSub.set(PodConfig.kP);
			azimuthkISub.set(PodConfig.kI);
			azimuthkDSub.set(PodConfig.kD);

			azimuthkDSub.getAtomic();
		}

		if (Robot.isSimulation()) {
			gyroSimState = new Pigeon2SimState(gyro);
		}
	}

	@Override
	public void periodic() {
		//update odometry
		poseEstimator.update(getGyro(), getModulePositions());

		//update telemetry
		SmartDashboard.putNumber("pigeon", getGyro().getDegrees());

		field2d.setRobotPose(getPose());

		SMSPublisher.set(getModuleStates());
		PosePublisher.set(getPose());
		ChassisSpeedsPublisher.set(RobotMap.drivetrainKinematics.toChassisSpeeds(getModuleStates()));


		// update the numbers (if tuning mode is enabled)
		if (Constants.tuningMode) {
			pods.forEach(pod -> {
			pod.setPID(
					azimuthkPSub.get(),
					azimuthkISub.get(),
					azimuthkDSub.get());
		});
		}
	}

	private void configurePathPlanner() {
        AutoBuilder.configure(
                this::getPose, // Supplier of current robot pose
                this::resetOdometry, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (chassisSpeeds) -> setRobotSpeeds(chassisSpeeds, false), // Consumer for setting robot chassis speeds
                new PPHolonomicDriveController(RobotConstants.TRANSLATION_PID, RobotConstants.ROTATION_PID),
                RobotConstants.CONFIG,
                () -> true,
                this); // Subsystem for requirements
    }

	private ChassisSpeeds getCurrentRobotChassisSpeeds() {
		// get the current robot chassis speeds
		return RobotMap.drivetrainKinematics.toChassisSpeeds(getModuleStates());
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {
		// odoAngleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI
		// * 0.5 : Math.PI * 1.5;
		// TODO: figure out why we were resetting odoAngleOffset 90 degrees off
		poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
	}

	public SwerveModulePosition[] getModulePositions() {
		return pods.stream()
				.map(DrivePod::getPodPosition)
				.toArray(SwerveModulePosition[]::new);
	}

	public SwerveModuleState[] getModuleStates() {
		return pods.stream()
				.map(DrivePod::getState)
				.toArray(SwerveModuleState[]::new);
	}

	/**
	 * resets the pods and odometry zero
	 */
	public void resetPods() {
		resetGyro();
		pods.stream().forEach(DrivePod::resetPod);

		resetOdometry(new Pose2d());
	}

	/**
	 * sets all motor output to 0
	 */
	public void stop() {
		pods.stream().forEach(DrivePod::stop);
	}

	public Rotation2d getGyro() {
		return gyro.getRotation2d();
	}

	public void resetGyro() {
		gyro.setYaw(0);
	}

	/**
	 * sets the robot's chassis speeds based on the given chassis speeds, enables isTurning
	 * @param chassisSpeeds the desired chassis speeds (+x forward, +y left, +omega counter-clockwise)
	 */
	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds) {
		setRobotSpeeds(chassisSpeeds, true);
	}

	/**
	 * sets the robot's chassis speeds based on the given chassis speeds
	 * @param chassisSpeeds the desired chassis speeds (+x forward, +y left, +omega counter-clockwise)
	 * @param enableIsTurning if true, the robot will not move if any pod is turning and will not start turning if any pod is still moving
	 */
	public void setRobotSpeeds(ChassisSpeeds chassisSpeeds, boolean enableIsTurning) {
		// for field-relative driving
		// chassisSpeeds =
		// ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
		// chassisSpeeds.vyMetersPerSecond,
		// chassisSpeeds.omegaRadiansPerSecond,
		// getGyro());


		SwerveModuleState[] states = RobotMap.drivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConstants.robotMaxLinearSpeed);

		for (int i = 0; i < pods.size(); i++) {
			pods.get(i).setPodState(states[i], enableIsTurning);
		}
	}

	public ArrayList<DrivePod> getPods() {
		return pods;
	}

	@Override
	public void simulationPeriodic() {
		// update the gyro to functionally be the robot/odo angle
		simGyroPosition += Units.radiansToDegrees(RobotMap.drivetrainKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond) * Robot.kDefaultPeriod;
		gyroSimState.setRawYaw(simGyroPosition);
	}

	/**
	 * initializes the suppliers for each pod to share information, specifically the
	 * global output scalar (for normalization)
	 * and whether any pod is turning (for safe/slow turning)
	 * 
	 * this MUST be called directly after initizliation
	 */
	private void initializeSuppliers() {
		DoubleSupplier globalOutputScalar = () -> pods.stream()
				.mapToDouble(DrivePod::getPodOutputScalar)
				.max()
				.orElse(0);

		BooleanSupplier isTurningSupplier = () -> pods.stream().anyMatch(DrivePod::isTurning);

		for (DrivePod pod : pods) {
			pod.initializeSuppliers(globalOutputScalar, isTurningSupplier);
		}
	}

	/**
	 * accepts a vision measurement for pose estimation
	 * 
	 * @param visionPose estimated pose from the vision system
	 * @param distance   distance from the vision target to the robot, used for
	 *                   uncertainty in the pose estimation
	 */
	public void addVisionMeasurement(EstimatedRobotPose visionPose, double distance) {
		poseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(),
				Utils.fpgaToCurrentTime(visionPose.timestampSeconds),
				// just a hardcoded /2 for now, this can be tuned further. This just decreases pose certainty as distance increases.
				VecBuilder.fill(distance / 2, distance / 2, distance / 2));
	}
}
