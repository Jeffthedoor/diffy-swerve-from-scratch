// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurdSwerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndependentDrive extends Command {

  private final TurdSwerve swerve;
  private final Supplier<Pose2d> leftJoystickVelocity, rightJoystickVelocity;
  private Pose2d formationCenterPosition = new Pose2d();
  /** Creates a new IndependentDrive, the archnemesis of TandemDrive */
  public IndependentDrive(TurdSwerve swerve, Supplier<Pose2d> leftJoystickVelocity, Supplier<Pose2d> rightJoystickVelocity) {
    this.swerve = swerve;
    this.leftJoystickVelocity = leftJoystickVelocity;
    this.rightJoystickVelocity = rightJoystickVelocity;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d velocity = Constants.IS_MASTER ? leftJoystickVelocity.get() : rightJoystickVelocity.get();
    swerve.setRobotSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), velocity.getY(), velocity.getRotation().getRadians(), swerve.getPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
