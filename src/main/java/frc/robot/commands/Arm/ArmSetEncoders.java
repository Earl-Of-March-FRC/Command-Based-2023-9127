// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetEncoders extends InstantCommand {

  ArmSubsystem armSub;
  double shoulderTicks;
  double elbowTicks;

  public ArmSetEncoders(ArmSubsystem aSub, double shoulderTicks, double elbowTicks) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = aSub;
    this.shoulderTicks = shoulderTicks;
    this.elbowTicks = elbowTicks;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.setEncoders(shoulderTicks, elbowTicks);
  }
}
