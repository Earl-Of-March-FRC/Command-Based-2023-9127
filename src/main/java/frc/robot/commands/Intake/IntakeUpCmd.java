package frc.robot.commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUpCmd extends CommandBase {
  IntakeSubsystem intakeSub;

  public IntakeUpCmd(IntakeSubsystem intakeSub) {
    this.intakeSub = intakeSub;
    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {
    System.out.println("Intake Up Command Started!");
  }

  @Override
  public void execute() {
    if(!intakeSub.coneRotationState){
      intakeSub.setIntake(-0.5);
    }else{
      end(true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake Up Command Ended");
    intakeSub.setIntake(0);
    intakeSub.coneRotationState = true;
  }

  @Override
  public boolean isFinished() {
    return !intakeSub.hitHallEffect();
  }
}
