// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Routines;

import java.sql.Driver;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Auto.IndividualCommands.LeaveCommunity;
import frc.robot.commands.Auto.IndividualCommands.LeaveCommunityBackwards;
import frc.robot.commands.Auto.IndividualCommands.MoveForwardProfile;
import frc.robot.commands.Auto.IndividualCommands.TurnDegrees;
import frc.robot.commands.Intake.TripperDownCmd;
import frc.robot.commands.Intake.TripperUpCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndGrab extends SequentialCommandGroup {

  /** Creates a new ScoreAndGrab. */
  public ScoreAndGrab(DrivetrainSubsystem dSub, IntakeSubsystem iSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new TripperDownCmd(iSub).withTimeout(2).andThen(new TripperUpCmd(iSub, -100).withTimeout(2)).andThen(new LeaveCommunityBackwards(dSub))
    );
  }
}
