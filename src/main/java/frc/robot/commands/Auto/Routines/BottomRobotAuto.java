package frc.robot.commands.Auto.Routines;
// package frc.robot.commands.Auto;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.Arm.ClawIn;
// import frc.robot.commands.Arm.ClawOut;
// import frc.robot.commands.Arm.RetractArm;
// import frc.robot.commands.Intake.IntakeDownCmd;
// import frc.robot.commands.Intake.IntakeUpCmd;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;

// public class BottomRobotAuto extends SequentialCommandGroup {
//   public BottomRobotAuto(DrivetrainSubsystem driveSub, ArmSubsystem armSub, IntakeSubsystem intakeSub) {
//     addCommands(
//       //new ClawIn(armSub).withTimeout(0.5), cone shouldn't fall out once it's already in the claw?
//       new ScoreHigh(armSub),
//       new RetractArm(armSub),
//       new MoveSideways(driveSub, -22), // might be positive to go to the left
//       new VisionHorizontalAlign(driveSub),
//       new MoveForward(driveSub, 185),
//       //new TurnDegrees(driveSub, 180),
//       new PickUpCone(intakeSub, driveSub, armSub),
//       new MoveForward(driveSub, -185),
//       new MoveSideways(driveSub, 22), // might be negative to go to the right
//       new VisionHorizontalAlign(driveSub),
//       new ScoreHigh(armSub),
//       new RetractArm(armSub)
//     );
//   }
// }
