// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.Arm.ArmMoveLowCube;
import frc.robot.commands.Arm.ArmSetEncoders;
import frc.robot.commands.Arm.RetractArm;
import frc.robot.commands.Arm.SpinClaw;
import frc.robot.commands.Auto.IndividualCommands.LeaveCommunityBackwards;

public class ScoreAndLeave extends SequentialCommandGroup {
  /** Creates a new ScoreAndLeave. */
  public ScoreAndLeave(DrivetrainSubsystem driveSub, ArmSubsystem armSub, ClawSubsystem clawSub) {
    addCommands(
      new ArmSetEncoders(armSub, 18, -195),
      new RetractArm(armSub, clawSub).withTimeout(1),
      new ArmMoveLowCube(armSub).withTimeout(3),
      new LeaveCommunityBackwards(driveSub).withTimeout(.2),
      new WaitCommand(1),
      new SpinClaw(clawSub, ClawConstants.CLAW_SPIN_OUT).withTimeout(3),
      new WaitCommand(1).alongWith(new SpinClaw(clawSub, 0)).withTimeout(1),
      new RetractArm(armSub, clawSub)
    );
  }
}
