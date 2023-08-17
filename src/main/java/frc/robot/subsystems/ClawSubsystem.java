package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ArmConstants;
 
public class ClawSubsystem extends SubsystemBase {
  private final WPI_TalonSRX clawMotor = new WPI_TalonSRX(ArmConstants.clawPort);

  public ClawSubsystem() {
    clawMotor.configPeakCurrentLimit(35, 10);
    clawMotor.configPeakCurrentDuration(200, 10);
    clawMotor.configContinuousCurrentLimit(20,10);
    clawMotor.enableCurrentLimit(true);
  }

  public void spinClaw(double speed) {
    System.out.println("spin claw command using value " + speed);
    clawMotor.set(speed);
  }

  @Override
  public void periodic() {
  }
}
