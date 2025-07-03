package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.DiffySwerve;

public class DriveCommand extends Command {

	private DiffySwerve swerve;
	private Supplier<Translation2d> joystickRight, joystickLeft;

	public DriveCommand(DiffySwerve swerve, Supplier<Translation2d> joystickLeft, Supplier<Translation2d> joystickRight) {
		this.swerve = swerve;
		this.joystickRight = joystickRight;
		this.joystickLeft = joystickLeft;
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		swerve.resetGyro();
	}

	@Override
	public void execute() {
		Translation2d joystickRightTranslation = joystickRight.get();
		Translation2d joystickLeftTranslation = joystickLeft.get();

		// x and y are swapped because WPILib has +x as forward for some reason
		// also joystick y is negated because thats how joysticks work
		double speedX = MathUtil.applyDeadband(-joystickRightTranslation.getY(), 0.05) * RobotConfig.robotMaxLinearSpeed;
		double speedY = MathUtil.applyDeadband(joystickRightTranslation.getX(), 0.05) * RobotConfig.robotMaxLinearSpeed;
		double speedOmega = MathUtil.applyDeadband(joystickLeftTranslation.getX(), 0.07) * RobotConfig.robotMaxRotationalSpeed;
		ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, speedOmega);

		swerve.setRobotSpeeds(speeds);

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
