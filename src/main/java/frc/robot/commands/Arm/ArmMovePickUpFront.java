package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmMovePickUpFront extends SequentialCommandGroup {
  /** Creates a new ArmMovePickUp. */
  public ArmMovePickUpFront(ArmSubsystem armSub, ClawSubsystem clawSub) {
    addCommands(
      new ArmMoveSetpoint(armSub, ArmConstants.SHOULDER_STOWED, ArmConstants.ELBOW_STOWED, ArmConstants.SHOULDER_STOWED_PID, ArmConstants.ELBOW_STOWED_PID, true)
        .withTimeout(2),
      new ArmMoveSetpoint(armSub, -12, 2900, ArmConstants.SHOULDER_PICKUP_PID, ArmConstants.ELBOW_PICKUP_PID, false)
        .withTimeout(3),
      new ArmFeedForward(armSub, ArmConstants.SHOULDER_PICKUP_FEEDFORWARD, -0.2)
        .alongWith(new SpinClaw(clawSub, ClawConstants.CLAW_SPIN_IN))
      //new ArmFeedForward(armSub, ArmConstants.SHOULDER_PICKUP_FEEDFORWARD, ArmConstants.ELBOW_PICKUP_FEEDFORWARD)
    );
  }
}