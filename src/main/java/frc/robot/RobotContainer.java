package frc.robot;
import frc.robot.commands.Arm.ArmMoveLowCube;
import frc.robot.commands.Arm.ArmMovePickUp;
import frc.robot.commands.Arm.ArmMovePickUpFront;
import frc.robot.commands.Arm.ArmMoveSetpoint;
import frc.robot.commands.Arm.ArmSetEncoders;
import frc.robot.commands.Arm.RetractArm;
import frc.robot.commands.Arm.SpinClaw;
import frc.robot.commands.Arm.TeleopArm;
import frc.robot.commands.Auto.Routines.ScoreAndGrab;
import frc.robot.commands.Auto.Routines.ScoreAndLeave;
import frc.robot.commands.Auto.Routines.autoElims;
import frc.robot.commands.Auto.IndividualCommands.LeaveCommunity;
import frc.robot.commands.Auto.IndividualCommands.LeaveCommunityBackwards;
import frc.robot.commands.Drivetrain.MecanumDriveCmd;
import frc.robot.commands.Drivetrain.ResetCalibrateGyro;
import frc.robot.commands.Intake.IntakeDownCmd;
import frc.robot.commands.Intake.IntakeUpCmd;
import frc.robot.commands.Intake.TripperDownCmd;
import frc.robot.commands.Intake.TripperUpCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawConstants;

public class RobotContainer {
  private final IntakeSubsystem inSub = new IntakeSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final ArmSubsystem armSub = new ArmSubsystem();
  public final ClawSubsystem clawSub = new ClawSubsystem();
  public final VisionSubsystem visionSub = new VisionSubsystem();

  private final Joystick mainJoystick = new Joystick(2);
  private final Joystick buttonPanel = new Joystick(1);
  private final Joystick operator = new Joystick(3);

  public RobotContainer() {
    driveSub.setDefaultCommand(new MecanumDriveCmd(driveSub, 
    () -> mainJoystick.getRawAxis(1),
    () -> mainJoystick.getRawAxis(0),
    () -> mainJoystick.getRawAxis(2)));

    armSub.setDefaultCommand(new TeleopArm(armSub, () -> -operator.getRawAxis(1), () -> -operator.getRawAxis(5)));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    // out
    new JoystickButton(operator, 4).onTrue(new IntakeUpCmd(inSub));
    //claw in
    new JoystickButton(operator, 3).onTrue(new IntakeDownCmd(inSub));
    //claw stop
    new JoystickButton(operator, 1).onTrue(new InstantCommand( () -> new ArmMoveLowCube(armSub).cancel()));
    new JoystickButton(buttonPanel, 2).onTrue(new ResetCalibrateGyro(driveSub));

    new JoystickButton(operator, 6).onTrue(new TripperDownCmd(inSub));
    new JoystickButton(operator, 2).onTrue(new ArmSetEncoders(armSub, 0, 0));
    new JoystickButton(operator, 5).onTrue(new TripperUpCmd(inSub, -100));
    new JoystickButton(buttonPanel, 3).onTrue(new IntakeDownCmd(inSub));
    new JoystickButton(buttonPanel,8).onTrue(new IntakeUpCmd(inSub));
    new POVButton(operator, 0).onTrue(new ArmMoveLowCube(armSub));
    new POVButton(operator, 90).onTrue(new ArmMovePickUpFront(armSub, clawSub));
    new POVButton(operator, 180).onTrue(new ArmMovePickUp(armSub, clawSub));
    new POVButton(operator, 270).onTrue(new RetractArm(armSub, clawSub));
    new JoystickButton(operator, 10).onTrue(new SpinClaw(clawSub, ClawConstants.CLAW_SPIN_IN));
    new JoystickButton(operator, 9).whileTrue(new SpinClaw(clawSub, ClawConstants.CLAW_SPIN_OUT)).whileFalse(new SpinClaw(clawSub, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int cycle) {
    // An example command will be run in autonomous
    switch(cycle){
      case 1:
        return new LeaveCommunityBackwards(driveSub);
      case 2:
        return new autoElims(clawSub, driveSub);
      case 3:
        return new LeaveCommunity(driveSub);
      case 4:
        return new ScoreAndGrab(driveSub, inSub);
      case 5:
        return new ScoreAndLeave(driveSub, armSub, clawSub);
    }
    return null;
  
  }
}