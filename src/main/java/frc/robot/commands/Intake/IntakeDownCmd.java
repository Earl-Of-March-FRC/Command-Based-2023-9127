package frc.robot.commands.Intake;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDownCmd extends CommandBase {
  IntakeSubsystem intakeSub;

  public IntakeDownCmd(IntakeSubsystem intakeSub) {
    this.intakeSub = intakeSub;
    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {
    System.out.println("Intake Down Command Started!");
  }

  @Override
  public void execute() {
    if(intakeSub.coneRotationState){
      intakeSub.setIntake(0.3);
    }else{
      end(true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake Down Command Ended");
    intakeSub.setIntake(0);
    intakeSub.coneRotationState = false;
  }

  @Override
  public boolean isFinished() {
    return !intakeSub.hitLimitSwitch();
  }
}
