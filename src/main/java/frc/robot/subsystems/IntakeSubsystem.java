package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

  //Define our motors
  public final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.intakePort);
  public final WPI_TalonSRX tripperMotor = new WPI_TalonSRX(IntakeConstants.tripperPort);
  private double angle = 0;
  //On robot, we have limit switches and hall effect (magnetic limit switch) sensors to tell when to stop our motor
  private final DigitalInput limitSwitch = new DigitalInput(0);
  private final DigitalInput hallEffect = new DigitalInput(1); // Cone Hall effect
  private final DigitalInput hallEffectTripper = new DigitalInput(2);
  public boolean coneRotationState = true; // False - down True - up
  public boolean tripperRotationState = true; // False - down True - up
  public IntakeSubsystem() {
    // As part of the motors, they have their own PID -> define PID here
    intakeMotor.config_kP(0, 0.1);
    intakeMotor.config_kI(0, 0);
    intakeMotor.config_kD(0, 0);

    // tripperMotor.config_kP(0, 0.2);
    // tripperMotor.config_kI(0, 0);
    // tripperMotor.config_kD(0, 0);

    // Tripper motor has an encoder: Connect the encoder to the motor
    tripperMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    tripperMotor.setSelectedSensorPosition(0);
    //Set default position to 0 ticks, and set it to brake while not moving
    tripperMotor.setNeutralMode(NeutralMode.Brake);

  //  intakeMotor.configPeakCurrentLimit(35, 10);
    //intakeMotor.configPeakCurrentDuration(200,10);
    //intakeMotor.configContinuousCurrentLimit(20, 10);
    //intakeMotor.enableCurrentLimit(true);

    tripperMotor.configPeakCurrentLimit(35, 10);
    tripperMotor.configPeakCurrentDuration(200,10);
    tripperMotor.configContinuousCurrentLimit(20, 10);
    tripperMotor.enableCurrentLimit(true);
  }

  // Set PID values for the tripper
  public void setPID(double P, double I, double D){
    tripperMotor.config_kP(0, P);
    tripperMotor.config_kI(0, I);
    tripperMotor.config_kD(0, D);
  }

  // Set the power of the intake
  public void setIntake(double speed){
    intakeMotor.set(speed);
  }

  public void setTripperPower(double power){
    angle = getTripperEncoderPosition()/4096 * 360; 
    //System.out.println(angle-90);
    tripperMotor.set(ControlMode.PercentOutput, power);
  }
  //Set the tripper to an encoder value
  public void setTripperEncoderValue(double ticks) {
    tripperMotor.set(ControlMode.Position, ticks);
    //tripperMotor.set(ControlMode.PercentOutput, 0.2);
  }

  // Current ticks of the tripper
  public double getTripperEncoderPosition(){
    return tripperMotor.getSelectedSensorPosition();
  }

  // Hit limit switch?
  public Boolean hitLimitSwitch(){
    if(limitSwitch.get()){
      return true;
    }
    return false;
  }

  // Hit cone hall effect?
  public Boolean hitHallEffect(){
    if(hallEffect.get()){
      return true;
    }
    return false;
  }

  public Boolean hitHallEffectTripper(){
    if (hallEffectTripper.get()){
      return true;
    }else{
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tripper ticks: ", getTripperEncoderPosition());
    SmartDashboard.putBoolean("Hall Effect: ", hitHallEffect());
    SmartDashboard.putBoolean("Limit Switch: ", hitLimitSwitch());
    SmartDashboard.putNumber("Tripper angle: ", angle-90);
  }

 // @Override
  //public void teleopInit() {
   // tripperMotor.setSelectedSensorPosition(0);
  //}
}
