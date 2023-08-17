package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class BalanceOnCharge extends CommandBase {
  private DrivetrainSubsystem dSub;
  private PIDController pid = new PIDController(
    DrivetrainConstants.NormalPID.kP,
    DrivetrainConstants.NormalPID.kI,
    DrivetrainConstants.NormalPID.kD
  );

  public BalanceOnCharge(DrivetrainSubsystem driveSub) {
    dSub = driveSub;
    pid.setSetpoint(0);
    pid.setTolerance(2);
    addRequirements(dSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = pid.calculate(dSub.getPitch());
    dSub.setMecanum(speed, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    dSub.setBrakeMode();
    dSub.setMecanum(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
