// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

/** This is a sample pod that uses a CANcoder and TalonFXes. */
public class DrivePod extends SubsystemBase {
	// public Encoder absoluteEncoder;
	public TalonFXS leftMotor;
	public TalonFXS rightMotor;
	
	private double offset;

	private AnalogEncoder encoder;

	TalonFXSConfiguration leftConfig = new TalonFXSConfiguration();
	TalonFXSConfiguration rightConfig = new TalonFXSConfiguration();

	// variable that determines whether or not to apply PID configurations to the
	// motor (defaults to true for initial application)
	// boolean apply = true;

	private final PIDController anglePID;


	public DrivePod(int absoluteEncoderID, int leftID, int rightID, boolean leftInvert, boolean rightInvert, double absoluteEncoderOffset,
			boolean encoderInvert, int ampLimit, boolean brake,
			double rampRate, double kP, double kI, double kD, double maxOut) {

		leftMotor = makeMotor(leftID, leftInvert, brake, ampLimit, rampRate, 1d, 1d, leftConfig);
		rightMotor = makeMotor(rightID, rightInvert, brake, ampLimit, rampRate, 1d, 1d, rightConfig);


		encoder = new AnalogEncoder(absoluteEncoderID);
		encoder.setInverted(false);	
		this.offset = absoluteEncoderOffset;

		anglePID = new PIDController(kP, kI, kD);


		resetPod();
	}

	/**
	 * Creates a new TurdonFX (please use this for drive motors only)
	 * 
	 * @param id                         CAN ID for the motor
	 * @param inverted                   true for CW+, false for CCW+
	 * @param isBrake                    true for brake, false for coast
	 * @param statorLimit                the stator current limit in amps
	 * @param rampRate                   time it takes for the motor to reach full
	 *                                   power from zero power in seconds
	 * @param ENCODER_TO_MECHANISM_RATIO ratio between the feedback encoder
	 *                                   (integrated for drive motors) and the
	 *                                   mechanism. this varies based on your gear
	 *                                   ratio
	 * @param ROTOR_TO_ENCODER_RATIO     ratio between the rotor and the feedback
	 *                                   encoder. this is usually 1 for drive motors
	 */
	public TalonFXS makeMotor(int id, boolean inverted, boolean isBrake, double statorLimit, double rampRate,
			double ENCODER_TO_MECHANISM_RATIO, double ROTOR_TO_ENCODER_RATIO, TalonFXSConfiguration driveConfig) {
		// I figured nobody had the guts to put a CANivore on a turdswerve, so i'm
		// leaving out the CAN bus parameter
		TalonFXS motor = new TalonFXS(id);

		// set neutral mode and inverts
		driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		driveConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

		// set current limits; supply current limits are hardcoded because they are
		// almost always the same
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveConfig.CurrentLimits.SupplyCurrentLimit = 70d;
		driveConfig.CurrentLimits.SupplyCurrentLowerLimit = 40d;
		driveConfig.CurrentLimits.SupplyCurrentLowerTime = 1d;

		driveConfig.CurrentLimits.StatorCurrentLimitEnable = statorLimit > 0;
		driveConfig.CurrentLimits.StatorCurrentLimit = statorLimit;

		// this is kind of bad code, but it's the easiest way to set a ramp rate
		// regardless of control type
		driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampRate;
		driveConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = rampRate;
		driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = rampRate;
		driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
		driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
		driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

		// set feedback ratios
		driveConfig.ExternalFeedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
		driveConfig.ExternalFeedback.RotorToSensorRatio = ROTOR_TO_ENCODER_RATIO;
		// the remote sensor defaults to internal encoder

		motor.getConfigurator().apply(driveConfig);

		return motor;
	}


	public void resetPod() {
		rightMotor.setPosition(0);
		leftMotor.setPosition(0);
		offset = -encoder.get();
	}

	public void resetZero() {
		offset = -encoder.get();
		resetPod();
	}

	// current position of absolute encoder in rotations
	private double getAngle() {
		return encoder.get() + offset;
	}

	// average position accross drive motors
	private double getPosition() {
		return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
	}

	public void stop() {
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	public SwerveModulePosition getPodPosition() {
		return new SwerveModulePosition(getPosition(),
				Rotation2d.fromRotations(getAngle()));
	}

	public void setPodState(SwerveModuleState state) {
		// make sure pod is inside of possible rotation zone
		while (state.angle.getRotations() > 1) {
			state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.minus(Rotation2d.k180deg.plus(Rotation2d.k180deg)));
		} 
		while (state.angle.getRotations() < -1) {
			state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(Rotation2d.k180deg.plus(Rotation2d.k180deg)));
		} 
		while (state.angle.getRotations() > RobotMap.podRotationUpperBound) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.minus(Rotation2d.k180deg));
		} 
		while (state.angle.getRotations() < RobotMap.podRotationLowerBound) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.plus(Rotation2d.k180deg));
		} 

		// TODO: optimize pod target heading based on current heading

		//initialize outputs to raw speed
		double leftOutput = state.speedMetersPerSecond; 
		double rightOutput = state.speedMetersPerSecond; 

		//adjust outputs based on the PID Controller
		double correction = anglePID.calculate(getAngle(), state.angle.getRotations());
		leftOutput -= correction;
		rightOutput += correction;

		//normalize outputs to be between -1 and 1
		double maxOutput = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
		if (maxOutput > 1) {
			leftOutput /= maxOutput;
			rightOutput /= maxOutput;
		}
		// set the motor outputs
		leftMotor.setControl(new DutyCycleOut(leftOutput));
		rightMotor.setControl(new DutyCycleOut(rightOutput));




		//i have no idea what this stuff down here is for honestly

		// speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond;

		// double error = (state.angle.getRadians() - absoluteEncoder.getAbsolutePosition().getValueAsDouble())
		// 		% (2 * Math.PI);
		// error = error > Math.PI ? error - 2 * Math.PI : error;
		// error = error < -Math.PI ? error + 2 * Math.PI : error;
		// error *= 180 / Math.PI;
	}

	public void setRotationalSpeed(double speed) {
		leftMotor.set(speed);
		rightMotor.set(-speed);
	}

	@Override
	public void periodic() {
		// TODO: dont use smartdashboard
		// SmartDashboard.putNumber("absolute encoder" + absoluteEncoder.getDeviceID(),
		// 		absoluteEncoder.getAbsolutePosition().getValueAsDouble());
		// SmartDashboard.putNumber("azimuth pose " + absoluteEncoder.getDeviceID(),
		// 		azimuthMotor.getPosition().getValueAsDouble());
		// SmartDashboard.putNumber("azimuth pose " + config.absoluteEncoderID,
		// azimuthMotor.);

		// SmartDashboard.putNumber("drive pos " + driveMotor.getDeviceId(),
		// driveEncoder.getPosition());
		// SmartDashboard.putNumber("azimuth.getAppliedOutput()" +
		// azimuthMotor.getDeviceId(), azimuthMotor.getAppliedOutput());
		// //getAppliedOutput());
	}
}
