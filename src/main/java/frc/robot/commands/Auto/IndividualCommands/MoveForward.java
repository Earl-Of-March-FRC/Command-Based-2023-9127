package frc.robot.commands.Auto.IndividualCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveForward extends CommandBase {
  private DrivetrainSubsystem dSub;

  private double desiredTicks;
  private double desiredRotation;
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
  private ProfiledPIDController profilePID = new ProfiledPIDController(
    DrivetrainConstants.NormalPID.kP, 
    DrivetrainConstants.NormalPID.kI, 
    DrivetrainConstants.NormalPID.kD, new TrapezoidProfile.Constraints(10000, 20000));
  private PIDController rotationPid = new PIDController(
    0.0001, 0, 0);

  public MoveForward(DrivetrainSubsystem driveSub, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    dSub = driveSub;
    // inches = ticks / ticksPerRevolution * inchesPerRevollution
    desiredTicks = inches * DrivetrainConstants.ticksPerRev / DrivetrainConstants.inchesPerRev;
    pid.setSetpoint(desiredTicks);
    pid.setTolerance(500);

    desiredRotation = dSub.getAngle();
    rotationPid.setSetpoint(desiredRotation);
    rotationPid.setTolerance(2);

    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dSub.resetEncoderTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //dSub.setMecanumRobotOriented(pid.calculate(dSub.getEncoderValue()), 0, rotationPid.calculate(dSub.getAngle()));
    dSub.setMecanumRobotOriented(profilePID.calculate(dSub.getEncoderValue(), desiredTicks), 0, 0);
    SmartDashboard.putNumber("setpoint: ", pid.getSetpoint());
    SmartDashboard.putNumber("calculated PID: ", pid.calculate(dSub.getEncoderValue()));
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
    if(profilePID.atGoal()) return true;
    return false;
  }
}
