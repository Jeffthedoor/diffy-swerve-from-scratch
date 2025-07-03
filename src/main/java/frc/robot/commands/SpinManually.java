// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffySwerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinManually extends Command {
  /** Creates a new TestPods. */
  private DiffySwerve swerve;
  private Supplier<Integer> pod;
  private Supplier<Double> command;
  public SpinManually(DiffySwerve swerve, Supplier<Integer> pod, Supplier<Double> command) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.pod = pod;
    this.command = command;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pod.get() != -1) {
      swerve.getPods().get(pod.get()).setRotationalSpeed(command.get());
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
    return pod.get() == -1;
  }
}
