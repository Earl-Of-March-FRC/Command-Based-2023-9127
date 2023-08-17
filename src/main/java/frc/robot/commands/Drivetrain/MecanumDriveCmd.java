package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.function.Supplier;

public class MecanumDriveCmd extends CommandBase {
  private DrivetrainSubsystem driveSubsystem;
  private Supplier<Double> forwardFunction, sideFunction, rotateFunction;

  public MecanumDriveCmd(
    DrivetrainSubsystem driveSubsystem,
    Supplier<Double> forwardFunction,
    Supplier<Double> sideFunction,
    Supplier<Double> rotateFunction
  ) {
    this.driveSubsystem = driveSubsystem;
    this.forwardFunction = forwardFunction;
    this.sideFunction = sideFunction;
    this.rotateFunction = rotateFunction;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.setBrakeMode();
  }

  @Override
  public void execute() {
    // calculations here

    double speedForward = calcVelocity(
      forwardFunction.get(),
      DrivetrainConstants.forwardScaleFactor,
      false, DrivetrainConstants.forwardDeadzone
    );

    double speedSide = calcVelocity(
      sideFunction.get(),
      DrivetrainConstants.sidewaysScaleFactor,
      true, DrivetrainConstants.sidewaysDeadzone
    );

    double speedRotate = calcVelocity(
      rotateFunction.get(),
      DrivetrainConstants.rotateScaleFactor,
      true, DrivetrainConstants.rotateDeadzone
    );

    driveSubsystem.setMecanum(speedForward, speedSide, speedRotate);
    //driveSubsystem.setMecanum(flPower, frPower, blPower, brPower);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public double calcVelocity(double rawInput, double scaleFactor, boolean reversed, double deadzone) {
    double velocity = rawInput * scaleFactor;

    if (reversed) {
      velocity *= -1;
    }

    if (Math.abs(velocity) < deadzone) {
      velocity = 0.0;
    }

    return velocity;
  }
}
