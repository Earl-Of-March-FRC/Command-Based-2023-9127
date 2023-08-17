package frc.robot.commands.Auto.IndividualCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnDegrees extends CommandBase {
  private double currentAngle;
  private double desiredAngle;
  private DrivetrainSubsystem driveSub;
  private PIDController pidController;

  public TurnDegrees(DrivetrainSubsystem dSub, double desAngle) {
    addRequirements(dSub); 
    currentAngle = dSub.getAngle();
    desiredAngle = desAngle;
    driveSub = dSub;

    pidController = new PIDController(
      DrivetrainConstants.RotationPID.kP,
      DrivetrainConstants.RotationPID.kI,
      DrivetrainConstants.RotationPID.kD
    );
  
    pidController.setSetpoint(this.desiredAngle);
    pidController.setTolerance(2);
  }

  @Override
  public void initialize() {
    driveSub.resetGyro();
  }

  @Override
  public void execute() {
    currentAngle = driveSub.getAngle();
    double speed = pidController.calculate(currentAngle);
    driveSub.setMecanum(0, 0, -speed);
    SmartDashboard.putNumber("gyro angle: ", driveSub.getAngle());
    SmartDashboard.putNumber("TurnDegrees calculate: ", speed);
  }  

  @Override
  public void end(boolean interrupted) {
    driveSub.setMecanum(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
