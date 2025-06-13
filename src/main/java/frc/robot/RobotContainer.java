// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PointAndDrive;
import frc.robot.commands.ResetZeroes;
import frc.robot.commands.SpinManually;
import frc.robot.subsystems.DiffySwerve;

public class RobotContainer {

	public static final XboxController driverRaw = new XboxController(Constants.driverPort);
	public static final CommandXboxController driverCommand = new CommandXboxController(Constants.driverPort);
	public static final DiffySwerve swerve = new DiffySwerve();

	public RobotContainer() {
		configureBindings();
		Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(driverRaw.getRightX(),
				driverRaw.getRightY());
		Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(driverRaw.getLeftX(),
				driverRaw.getLeftY());
		Supplier<Integer> DPAD = () -> driverRaw.getPOV();
		swerve.setDefaultCommand(
				new DriveCommand(swerve, driverLeftJoystick, driverRightJoystick, DPAD, driverRaw::getLeftBumperButton));


		// swerve.setDefaultCommand(new RunCommand(() -> swerve.getPods()[0].setPodState(new SwerveModuleState(driverRaw.getLeftY(), Rotation2d.fromDegrees(driverRaw.getPOV() == -1 ? 0 : driverRaw.getPOV()))), swerve));
	}

	private void configureBindings() {
		driverCommand.pov(-1).onFalse(new SpinManually(swerve, () -> getPodToTest(), () -> driverRaw.getRightX()));
		driverCommand.rightTrigger().whileTrue(new PointAndDrive(swerve, () -> new Translation2d(driverRaw.getRightX(), driverRaw.getRightY()), () -> driverRaw.getRightTriggerAxis()));

		driverCommand.rightBumper().and(driverRaw::getYButton).onTrue(new ResetZeroes(swerve));
		// // driverCommand.rightBumper().and(driverRaw::getXButton).whileTrue(new RevertZeroes(swerve));
		driverCommand.start().whileTrue(new InstantCommand(swerve::resetPods, swerve));
	}
	
	private int getPodToTest() {
		switch (driverRaw.getPOV()) {
			case 0:
				return 0;
			case 90:
				return 1;
			case 180:
				return 2;
			case 270:
				return 3;
			default:
				return -1;
		}
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
