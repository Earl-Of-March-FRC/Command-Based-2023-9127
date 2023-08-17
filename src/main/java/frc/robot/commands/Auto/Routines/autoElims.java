package frc.robot.commands.Auto.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SpinClaw;
import frc.robot.commands.Auto.IndividualCommands.LeaveCommunity;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.ClawConstants;

public class autoElims extends SequentialCommandGroup{
    public autoElims(ClawSubsystem clawSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        addCommands(
            new SpinClaw(clawSubsystem, ClawConstants.CLAW_SPIN_OUT).withTimeout(2),
            new SpinClaw(clawSubsystem, 0.0).withTimeout(2),
            new LeaveCommunity(drivetrainSubsystem)
        );
    }
}
