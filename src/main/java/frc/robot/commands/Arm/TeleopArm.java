// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
public class TeleopArm extends CommandBase {
  ArmSubsystem aSub;
  Supplier<Double> shoulderFunction, elbowFunction;
  double shoulderPower = 0.0001, elbowPower = 0.0001;
  /** Creates a new TeleopArm. */
  public TeleopArm(ArmSubsystem armSub, Supplier<Double> shoulder, Supplier<Double> elbow) {
    aSub = armSub;
    shoulderFunction = shoulder;
    elbowFunction = elbow;
    addRequirements(aSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public TeleopArm(ArmSubsystem armSub, double shoulder, double elbow){
    aSub = armSub;
    shoulderPower = shoulder;
    elbowPower = elbow;
    addRequirements(aSub);
  }
// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aSub.teleopShoulder(shoulderFunction.get());
    aSub.teleopElbow(elbowFunction.get());
    SmartDashboard.putNumber(("elbow power "), elbowPower);
    SmartDashboard.putNumber("shoulder power", shoulderPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
