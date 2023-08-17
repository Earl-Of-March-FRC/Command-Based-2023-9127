package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.controller.PIDController;
public class TripperUpCmd extends CommandBase {
  IntakeSubsystem intakeSub;
  double ticksNeeded;
  private PIDController pid = new PIDController(0.00069, 0, 0);
  public TripperUpCmd(IntakeSubsystem intakeSub, double ticks) {
    this.intakeSub = intakeSub;
    ticksNeeded = ticks;
    pid.setSetpoint(ticks);
    pid.setTolerance(5);
    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {
    //intakeSub.setPID(0.3, 0, 0);
    System.out.println("Tripper Up Command Started!");
  }

  @Override
  public void execute() {
    
    // if(!intakeSub.tripperRotationState){
    //   intakeSub.setTripperPower(0.3);
    // }else{
    //   end(true);
    // }
    //intakeSub.setTripperEncoderValue(ticksNeeded);
    intakeSub.setTripperPower(-pid.calculate(intakeSub.getTripperEncoderPosition()));
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Tripper Up Command Ended");
    intakeSub.setTripperPower(0);
    intakeSub.tripperRotationState = true;
  }

  @Override
  public boolean isFinished() {
    // return pid.atSetpoint();
    return !intakeSub.hitHallEffectTripper();
  }
}