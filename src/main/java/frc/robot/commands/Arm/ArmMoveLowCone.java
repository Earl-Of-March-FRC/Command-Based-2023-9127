package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveLowCone extends CommandBase {
  ArmSubsystem armSub;

  public ArmMoveLowCone(ArmSubsystem armSub) {
    this.armSub = armSub;
    addRequirements(armSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    //armSub.setArm(ArmConstants.targetsX[2], ArmConstants.targetsY[2]);
    //armSub.setEncoderSetpoints(150, 500);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}