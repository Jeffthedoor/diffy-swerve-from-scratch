// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TurdConstants.RobotConfig;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.TurdSwerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndependentDrive extends Command {
  public static Pose2d masterOffset = RobotConfig.offsetPositions[0];
  private final TurdSwerve swerve;
  private final Supplier<Pose2d> leftJoystickVelocity, rightJoystickVelocity, masterOffsetSupplier;
  // private Pose2d formationCenterPosition = new Pose2d();
  /** Creates a new IndependentDrive, the archnemesis of TandemDrive */
  public IndependentDrive(TurdSwerve swerve, Supplier<Pose2d> leftJoystickVelocity, Supplier<Pose2d> rightJoystickVelocity, Supplier<Pose2d> masterOffsetSupplier) {
    this.swerve = swerve;
    this.leftJoystickVelocity = leftJoystickVelocity;
    this.rightJoystickVelocity = rightJoystickVelocity;
    this.masterOffsetSupplier = masterOffsetSupplier;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // formationCenterPosition = 
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
    if (!Constants.IS_MASTER) {
      Pose2d masterPose = PhotonVision.closestMasterPose;
      Pose2d formationCenterPose = new Pose2d(masterPose.getTranslation().plus(swerve.getPose().getTranslation()).times(0.5), masterPose.getRotation());
      masterOffset = new Pose2d(masterPose.getTranslation().minus(formationCenterPose.getTranslation()).rotateBy(masterPose.getRotation().times(-1)), new Rotation2d());
      RobotConfig.offsetPositions[1] = new Pose2d(swerve.getPose().getTranslation().minus(formationCenterPose.getTranslation()).rotateBy(formationCenterPose.getRotation()), swerve.getPose().getRotation().minus(formationCenterPose.getRotation()));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
