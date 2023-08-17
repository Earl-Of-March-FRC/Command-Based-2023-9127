// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveLowCube extends SequentialCommandGroup {
  /** Creates a new ArmMoveLowCube. */
  public ArmMoveLowCube(ArmSubsystem armSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmMoveSetpoint(armSub, ArmConstants.SHOULDER_LOW_CUBE_SETPOINT, ArmConstants.ELBOW_LOW_CUBE_SETPOINT, ArmConstants.SHOULDER_LOW_CUBE_PID, ArmConstants.ELBOW_LOW_CUBE_PID, false)
        .withTimeout(15),
      new ArmFeedForward(armSub, ArmConstants.SHOULDER_LOW_CUBE_FEEDFORWARD, ArmConstants.ELBOW_LOW_CUBE_FEEDFORWARD)
    );
  }
}
