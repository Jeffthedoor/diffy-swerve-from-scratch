// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants;
import frc.robot.Robot;

/** This is a sample pod that uses a CANcoder and TalonFXes. */
public class DrivePod extends SubsystemBase {
	// public Encoder absoluteEncoder;
	private final TalonFXS leftMotor;
	private final TalonFXS rightMotor;
	private final CANcoder encoder;
	
	private TalonFXSSimState leftMotorSim;
	private TalonFXSSimState rightMotorSim;
	private CANcoderSimState absoluteEncoderSim;
	private DifferentialDrivetrainSim drivetrainSim;
	
	private DoublePublisher podScalarPublisher;
	private DoublePublisher encoderAnglePublisher;
	private DoublePublisher leftPositionPublisher;
	private DoublePublisher rightPositionPublisher;
	private DoublePublisher leftOutputPublisher;
	private DoublePublisher rightOutputPublisher;

	private double leftMotorOutput = 0;
	private double rightMotorOutput = 0;
	
	private double offset;



	SwerveModuleState lastGoodTargetState = new SwerveModuleState(0, Rotation2d.fromRotations(0));
	double zone = 10;
	
	private double podOutputScalar = 1;
	private DoubleSupplier globalOutputScalar;
	private BooleanSupplier otherPodTurning;
	
	TalonFXSConfiguration leftConfig = new TalonFXSConfiguration();
	TalonFXSConfiguration rightConfig = new TalonFXSConfiguration();
	CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

	private final PIDController anglePID;

	private boolean isTurning = false;

	/**
	 * Creates a DrivePod with the specified parameters.
	 * @param podID The ID of the pod (used only for logging).
	 * @param absoluteEncoderID The ID of the absolute encoder.
	 * @param leftID The CAN ID of the left motor.
	 * @param rightID The CAN ID of the right motor.
	 * @param leftInvert Whether the left motor is inverted.
	 * @param rightInvert Whether the right motor is inverted.
	 * @param absoluteEncoderOffset The offset for the absolute encoder.
	 * @param encoderInvert Whether the absolute encoder is inverted.
	 * @param ampLimit The current limit for the motors.
	 * @param brake Whether the motors should be in brake mode.
	 * @param rampRate The ramp rate for the motors.
	 * @param kP The proportional gain for the angle PID controller.
	 * @param kI The integral gain for the angle PID controller.
	 * @param kD The derivative gain for the angle PID controller.
	 * @param globalMaxOutput A supplier for the global maximum output scalar.
	 * @param motorGearing The gearing ratio of the motors.
	 */
	public DrivePod(int podID, int absoluteEncoderID, int leftID, int rightID, boolean leftInvert, boolean rightInvert, double absoluteEncoderOffset,
			boolean encoderInvert, int ampLimit, boolean brake,
			double rampRate, double kP, double kI, double kD, double motorGearing) {

		leftMotor = makeMotor(leftID, leftInvert, brake, ampLimit, motorGearing, rampRate, leftConfig);
		rightMotor = makeMotor(rightID, rightInvert, brake, ampLimit, motorGearing, rampRate, rightConfig);

		encoder = makeEncoder(absoluteEncoderID, encoderInvert, absoluteEncoderOffset);


		anglePID = new PIDController(kP, kI, kD);
		// anglePID.enableContinuousInput(0, 1);


		resetPod();
		startLogging(podID);

		if(Robot.isSimulation()) {
			makeSim();
		}
	}
	/** 
	 * sets local suppliers to global suppliers
	 * @param globalMaxOutput normalization factor for the whole swerve drive
	 * @param otherPodTurning whether a pod is currently turning a large angle (used to determine if the robot should stop moving before turning)
	 */
	public void initializeSuppliers(DoubleSupplier gloabalMaxOutput, BooleanSupplier otherPodTurning) {
		this.globalOutputScalar = gloabalMaxOutput;
		this.otherPodTurning = otherPodTurning;
	}

	/**
	 * Creates a TalonFXS motor with the given parameters.
	 * @param id The CAN ID of the motor.
	 * @param inverted Whether the motor is inverted.
	 * @param isBrake Whether the motor should be in brake mode.
	 * @param statorLimit The stator current limit for the motor.
	 * @param motorGearing The gearing ratio of the motor.
	 * @param rampRate The ramp rate for the motor.
	 * @param driveConfig The configuration for the motor.
	 * @return A configured TalonFXS motor.
	 */
	private TalonFXS makeMotor(int id, boolean inverted, boolean isBrake, double statorLimit, double motorGearing, double rampRate, TalonFXSConfiguration driveConfig) {
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
		driveConfig.ExternalFeedback.SensorToMechanismRatio = motorGearing;
		driveConfig.ExternalFeedback.RotorToSensorRatio = 1d;

		motor.getConfigurator().apply(driveConfig);

		return motor;
	}


    /**
     * creates a new CANCoder
     * @param id the CAN id of the sensor
     * @param inverted true for CW+, false for CCW+
     * @param offset the offset of the sensor in rotations
     */
    private CANcoder makeEncoder(int id, boolean inverted, double offset) {
        CANcoder encoder = new CANcoder(id);

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = offset;

        encoder.getConfigurator().apply(encoderConfig);

        return encoder;
    }

	private void startLogging(int id) {
		podScalarPublisher = NetworkTableInstance.getDefault().getTable("pod states").getSubTable("pod" + id).getDoubleTopic("OutputScalar").publish();
		encoderAnglePublisher = NetworkTableInstance.getDefault().getTable("pod states").getSubTable("pod" + id).getDoubleTopic("EncoderAngle").publish();
		leftPositionPublisher = NetworkTableInstance.getDefault().getTable("pod states").getSubTable("pod" + id).getDoubleTopic("LeftPosition").publish();
		rightPositionPublisher = NetworkTableInstance.getDefault().getTable("pod states").getSubTable("pod" + id).getDoubleTopic("RightPosition").publish();
		leftOutputPublisher = NetworkTableInstance.getDefault().getTable("pod states").getSubTable("pod" + id).getDoubleTopic("LeftOutput").publish();
		rightOutputPublisher = NetworkTableInstance.getDefault().getTable("pod states").getSubTable("pod" + id).getDoubleTopic("RightOutput").publish();
		updateLogging();
	}

	private void updateLogging() {
		podScalarPublisher.set(podOutputScalar);
		encoderAnglePublisher.set(getAngle());
		leftPositionPublisher.set(leftMotor.getPosition().getValueAsDouble());
		rightPositionPublisher.set(rightMotor.getPosition().getValueAsDouble());
		leftOutputPublisher.set(leftMotorOutput);
		rightOutputPublisher.set(rightMotorOutput);
	}

	/**
	 * Creates a simulation of the drivetrain.
	 * This method initializes the simulation states for the left and right motors,
	 * and sets the motor orientations according to the drivetrain configuration.
	 */
	private void makeSim() {
		leftMotorSim = leftMotor.getSimState();
		rightMotorSim = rightMotor.getSimState();
		absoluteEncoderSim = encoder.getSimState();


		// left drivetrain motors are typically CCW+
		leftMotorSim.MotorOrientation = ChassisReference.CounterClockwise_Positive;

		// right drivetrain motors are typically CW+
		rightMotorSim.MotorOrientation = ChassisReference.Clockwise_Positive;

		drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
			KitbotMotor.kSingleMiniCIMPerSide, // 2 CIMs per side.
			KitbotGearing.k10p71,        // 10.71:1
			KitbotWheelSize.kSixInch,    // 6" diameter wheels.
			3,
			null                         // No measurement noise.
		);

		//TODO: ensure stall torque, free current, and stall current are correct; these values are guesstimates
		// drivetrainSim = new DifferentialDrivetrainSim(new DCMotor(16, 300, 20, 2, RotationsPerSecond.of(27).in(RadiansPerSecond), 1),
		// 		SimConstants.gearRatio,
		// 		SimConstants.inertia,
		// 		SimConstants.mass, 
		// 		SimConstants.wheelRadius,
		// 		SimConstants.trackWidth,
		// 		VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
	}

	/** resets the pod zero */
	public void resetPod() {
		rightMotor.setPosition(0);
		leftMotor.setPosition(0);
		offset = -(getAngle() - offset);
	}

	// current position of absolute encoder in rotations
	private double getAngle() {
		return encoder.getAbsolutePosition().getValueAsDouble();
	}

	// average position accross drive motors
	private double getPosition() {
		// return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
		return (drivetrainSim.getLeftPositionMeters() + drivetrainSim.getRightPositionMeters()) / 2;
		//TODO: for the love of god change this before it goes on a robot
	}

	private double getVelocity() {
		// return (leftMotor.getVelocity().getValueAsDouble() + rightMotor.getVelocity().getValueAsDouble()) / 2;
		return (drivetrainSim.getLeftVelocityMetersPerSecond() + drivetrainSim.getRightVelocityMetersPerSecond()) / 2;
		//TODO: for the love of god change this before it goes on a robot
	}

	public void stop() {
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	public SwerveModulePosition getPodPosition() {
		return new SwerveModulePosition(getPosition(),
				Rotation2d.fromRotations(getAngle()));
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(),
				Rotation2d.fromRotations(getAngle()));
	}

	public void setPID(double kP, double kI, double kD) {
		anglePID.setPID(kP, kI, kD);
	}

	public SwerveModuleState optimizePodHeading(SwerveModuleState state) {
		state = new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromRotations(state.angle.getRotations() % 1));
		if (state.angle.getRotations() < -0.5) {
			state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(Rotation2d.fromRotations(1)));
		} else if (state.angle.getRotations() > 0.5) {
			state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.minus(Rotation2d.fromRotations(1)));
		}
		if (state.angle.getRotations() - getState().angle.getRotations() > 0.25) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.minus(Rotation2d.fromRotations(0.5)));
		} else if (state.angle.getRotations() - getState().angle.getRotations() < -0.25) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.plus(Rotation2d.fromRotations(0.5)));
		} 
		// make sure pod is inside of possible rotation zone
		if (state.angle.getRotations() > RobotMap.podRotationUpperBound) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.minus(Rotation2d.fromRotations(0.5)));
		} else if (state.angle.getRotations() < RobotMap.podRotationLowerBound) {
			state = new SwerveModuleState(-state.speedMetersPerSecond, state.angle.plus(Rotation2d.fromRotations(0.5)));
		} 
		return state;
	}

	private SwerveModuleState shouldSlowDown(SwerveModuleState targetState) {
		isTurning = Math.abs(targetState.angle.minus(getState().angle).getDegrees()) > zone;

		//large turn is requested
		if(isTurning || otherPodTurning.getAsBoolean()) {
			if(isMoving()) {
				//if pod in motion, slow down first with pod at current angle (or last known good angle)
				targetState = new SwerveModuleState(0d, lastGoodTargetState.angle);
			} else {
				//if pod not in motion, turn to target angle without moving
				targetState = new SwerveModuleState(0d, targetState.angle);
			}

			zone = 10;
		} else {
			//cache last known "good" target state
			lastGoodTargetState = targetState;
			zone = 45;
		}

		return targetState;
	}

	public void setPodState(SwerveModuleState state) {
		setPodState(state, true);
	}

	/** 
	 * sets the pod state
	 * @param enableIsTurning if true, the robot will not move if any pod is turning and will not start turning if any pod is still moving
	 */
	public void setPodState(SwerveModuleState state, boolean enableIsTurning) {
		// optimize pod target heading based on current heading
		state = optimizePodHeading(state);
		if (enableIsTurning) {
			state = shouldSlowDown(state);
		}

		//initialize outputs to raw speed
		double leftOutput = state.speedMetersPerSecond / RobotConstants.robotMaxLinearSpeed; 
		double rightOutput = state.speedMetersPerSecond / RobotConstants.robotMaxLinearSpeed;; 

		//adjust outputs based on the PID Controller
		double correction = anglePID.calculate(getAngle(), state.angle.getRotations());
		leftOutput += correction;
		rightOutput -= correction;

		
		//prevent race conditions
		double globalScalar = globalOutputScalar.getAsDouble();	
		double podMaxOutput = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));


		//if the pod output is greater than what the motor can output, scale it down
		if (podMaxOutput > RobotConstants.motorMaxOutput) {
			podOutputScalar = RobotConstants.motorMaxOutput / podMaxOutput;

			//if the calculated power scalar for the pod is greater than the global scalar, scale the outputs down by the pod scalar
			//otherwise, scale the outputs down by the global scalar. It is assumed that the global scalar will update within a loop cycle.
			//it is also assumed that all scalars are 1 during non-turning operations.
			//TODO: log each podOutputScalar
			if(podOutputScalar > globalScalar) {
				leftOutput *= podMaxOutput;
				rightOutput *= podMaxOutput;
			} else {
				leftOutput *= globalScalar;
				rightOutput *= globalScalar;
			}
		}
		
		
		// set the motor outputs
		leftMotor.setControl(new DutyCycleOut(leftOutput));
		rightMotor.setControl(new DutyCycleOut(rightOutput));

		//update class-scope variables for logging
		leftMotorOutput = leftOutput;
		rightMotorOutput = rightOutput;



		//i have no idea what this stuff down here is for honestly

		// speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond;

		// double error = (state.angle.getRadians() - absoluteEncoder.getAbsolutePosition().getValueAsDouble())
		// 		% (2 * Math.PI);
		// error = error > Math.PI ? error - 2 * Math.PI : error;
		// error = error < -Math.PI ? error + 2 * Math.PI : error;
		// error *= 180 / Math.PI;
	}

	public double getPodOutputScalar() {
		return podOutputScalar;
	}

	public void setRotationalSpeed(double speed) {
		leftMotor.set(speed);
		rightMotor.set(-speed);
	}


	@Override
	public void periodic() {
		updateLogging();
	}

	// public boolean isTurning() {
	// 	return isMoving() ? Math.abs(targetState.angle.minus(getState().angle).getRotations()) > 0.1 : Math.abs(targetState.angle.minus(getState().angle).getRotations()) > 0.03;
	// }

	public boolean isMoving() {
		return Math.abs(getState().speedMetersPerSecond) > 0.5;
	}

	public boolean isTurning() {
		return isTurning;
	}

	@Override
	public void simulationPeriodic() {
		leftMotorSim.setSupplyVoltage(16);
		rightMotorSim.setSupplyVoltage(16);

		//TODO: might not have to invert the left motor voltage
		drivetrainSim.setInputs(
				leftMotorSim.getMotorVoltage(),
				rightMotorSim.getMotorVoltage());
		drivetrainSim.update(0.02); // 20ms update rate, typical for simulation
		double metersToMotorRotations = SimConstants.gearRatio / (2 * Math.PI * SimConstants.wheelRadius);
		leftMotorSim.setRawRotorPosition(drivetrainSim.getLeftPositionMeters() * metersToMotorRotations);
		rightMotorSim.setRawRotorPosition(drivetrainSim.getRightPositionMeters() * metersToMotorRotations);
		leftMotorSim.setRotorVelocity(drivetrainSim.getLeftVelocityMetersPerSecond() * metersToMotorRotations);
		rightMotorSim.setRotorVelocity(drivetrainSim.getRightVelocityMetersPerSecond() * metersToMotorRotations);

		absoluteEncoderSim.setRawPosition(MathUtil.inputModulus(drivetrainSim.getHeading().getRotations(), -1,  1));
	}
}
