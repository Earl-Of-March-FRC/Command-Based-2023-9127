package frc.robot.commands.Auto.IndividualCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class MoveSideways extends CommandBase {
  private DrivetrainSubsystem dSub;

  private double desiredTicks;
  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
    DrivetrainConstants.NormalPID.kS,
    DrivetrainConstants.NormalPID.kV,
    DrivetrainConstants.NormalPID.kA
  );
  private PIDController pid = new PIDController(
    DrivetrainConstants.NormalPID.kP,
    DrivetrainConstants.NormalPID.kI,
    DrivetrainConstants.NormalPID.kD
  );

  public MoveSideways(DrivetrainSubsystem driveSub, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    dSub = driveSub;
    // inches = ticks / ticksPerRevolution * inchesPerRevollution
    desiredTicks = inches * DrivetrainConstants.ticksPerRev / DrivetrainConstants.inchesPerRev;
    pid.setSetpoint(desiredTicks);
    pid.setTolerance(2);
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dSub.setMecanum(0, pid.calculate(dSub.getEncoderValue()) + feedForward.calculate(dSub.getVelocity()), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dSub.setBrakeMode();
    dSub.setMecanum(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pid.atSetpoint()) return true;
    return false;
  }
}