// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffySwerve;
import frc.robot.subsystems.DrivePod;

public class PointAndDrive extends Command {

	DiffySwerve swerve;
	Supplier<Translation2d> joystickRight;
	Rotation2d angle = new Rotation2d();
	Supplier<Double> speed;

	public PointAndDrive(DiffySwerve swerve, Supplier<Translation2d> joystickRight, Supplier<Double> speed) {
		this.swerve = swerve;
		this.joystickRight = joystickRight;
		this.speed = speed;
		addRequirements(swerve);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (joystickRight.get().getNorm() > 0.5) {
			angle = joystickRight.get().getAngle().plus(Rotation2d.kCCW_90deg).times(-1); // TODO: test if this offset is correct
		}
		SwerveModuleState state = new SwerveModuleState(speed.get(), angle);
		for (DrivePod pod : swerve.getPods()) {
			pod.setPodState(state);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
