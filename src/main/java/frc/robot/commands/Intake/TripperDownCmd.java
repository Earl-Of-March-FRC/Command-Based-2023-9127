package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
public class TripperDownCmd extends CommandBase {
  IntakeSubsystem intakeSub;
  private PIDController pid = new PIDController(0.00065, 0, 0);
  public TripperDownCmd(IntakeSubsystem intakeSub) {
    this.intakeSub = intakeSub;
    pid.setSetpoint(960);
    pid.setTolerance(20);
    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {
    //intakeSub.setPID(0.1, 0, 0);
    System.out.println("Tripper Down Command Started!");

  }

  @Override
  public void execute() {
    intakeSub.setTripperPower(-pid.calculate(intakeSub.getTripperEncoderPosition()));
    //intakeSub.setTripperEncoderValue(-500);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSub.setTripperPower(0.08);
    System.out.println("Tripper Down Command Ended");
    intakeSub.tripperRotationState = false;
  }

  @Override
  public boolean isFinished() {
    //if (Math.abs(intakeSub.getEncoderPosition() - 4096) <= 5) return true;
    return pid.atSetpoint() || Math.abs(intakeSub.getTripperEncoderPosition() - 960) < 40;
  }
}