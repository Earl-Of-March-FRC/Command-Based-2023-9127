// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
public class ArmMoveSetpoint extends CommandBase {

  private ArmSubsystem armSub;
  private double shoulderSetpoint;
  private double elbowSetpoint;
  private double startTime;
  private PIDController elbowPid;
  private boolean elbowFirst;

  /** Creates a new ArmMoveSetpoint. */
  public ArmMoveSetpoint(ArmSubsystem aSub, double shoulder, double elbow, double[] shoulderPidValues, double[] elbowPidValues, boolean elbowFirst) {
    armSub = aSub;
    shoulderSetpoint = shoulder;
    elbowSetpoint = elbow;
    startTime = System.currentTimeMillis();
    this.elbowFirst = elbowFirst;

    aSub.setShoulderPID(shoulderPidValues);
    elbowPid = new PIDController(elbowPidValues[0], elbowPidValues[1], elbowPidValues[2]);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Convert ticks to angle
    elbowPid.setSetpoint(elbowSetpoint * ArmConstants.elbowTickAngleConv);
    elbowPid.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elbowFirst){
      armSub.teleopElbow(elbowPid.calculate(armSub.getElbowAngle()));
      if(System.currentTimeMillis() - startTime >= 1000){
        armSub.setShoulderSetpoint(shoulderSetpoint);
      }
    }
    else{
      armSub.setShoulderSetpoint(shoulderSetpoint);
      if(System.currentTimeMillis() - startTime >= 1000){
        armSub.teleopElbow(elbowPid.calculate(armSub.getElbowAngle()));
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elbowPid.atSetpoint() &&
        Math.abs(shoulderSetpoint - armSub.getShoulderEncoder()) < 2) {
      return true;
    }
    return false;
  }
}
