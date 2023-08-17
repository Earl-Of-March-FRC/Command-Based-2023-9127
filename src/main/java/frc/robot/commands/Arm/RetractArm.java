// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractArm extends SequentialCommandGroup {
  /** Creates a new RetractArm. */
  public RetractArm(ArmSubsystem aSub, ClawSubsystem cSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmMoveSetpoint(aSub, ArmConstants.SHOULDER_STOWED, ArmConstants.ELBOW_STOWED, 
        ArmConstants.SHOULDER_STOWED_PID, ArmConstants.ELBOW_STOWED_PID, true).alongWith(new SpinClaw(cSub, -0.15))
    );
  }
}
