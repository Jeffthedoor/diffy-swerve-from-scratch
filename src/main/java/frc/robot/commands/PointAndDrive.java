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
import frc.robot.subsystems.TurdSwerve;

public class PointAndDrive extends Command {

	TurdSwerve swerve;
	Supplier<Translation2d> joystickRight;
	Rotation2d angle = new Rotation2d();
	Supplier<Double> speed;
	/** 
	 * point the swerve pods in the direction of the joystick
	 * @param swerve the swerve subsystem
	 * @param joystickRight the direction to point in
	 * @param speed the speed to drive at, 0-1
	 */
	public PointAndDrive(TurdSwerve swerve, Supplier<Translation2d> joystickRight, Supplier<Double> speed) {
		this.swerve = swerve;
		this.joystickRight = joystickRight;
		this.speed = speed;
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		if (joystickRight.get().getNorm() > 0.5) {
			angle = joystickRight.get().getAngle().plus(Rotation2d.kCCW_90deg).times(-1); // TODO: test if this offset is correct
		}
		SwerveModuleState state = new SwerveModuleState(speed.get(), angle);

		swerve.getPods().stream().forEach(pod -> {
			pod.setPodState(state);
		});

	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
