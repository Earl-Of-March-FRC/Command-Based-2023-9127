// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.IndividualCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveForwardProfile extends ProfiledPIDCommand{
  /** Creates a new MoveForwardProfile. */
  public MoveForwardProfile(DrivetrainSubsystem driveSub, double inches) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            DrivetrainConstants.NormalPID.kP,
            DrivetrainConstants.NormalPID.kI,
            DrivetrainConstants.NormalPID.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(9500, 25000)),
        // This should return the measurement
        () -> driveSub.getEncoderValue(),
        // This should return the goal (can also be a constant)
        () -> (inches * DrivetrainConstants.ticksPerRev / DrivetrainConstants.inchesPerRev),
        // This uses the output
        (output, setpoint) -> driveSub.setMecanumRobotOriented(output, 0,0),
          driveSub);

        getController().setTolerance(50);

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
