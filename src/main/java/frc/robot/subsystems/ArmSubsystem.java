package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax shoulderMotor = new CANSparkMax(ArmConstants.shoulderPort, MotorType.kBrushless);
  private final CANSparkMax shoulderMotor2 = new CANSparkMax(ArmConstants.shoulderPort2, MotorType.kBrushless);
  private final WPI_TalonSRX elbowMotor = new WPI_TalonSRX(ArmConstants.elbowPort);
  private final SparkMaxPIDController shoulderController = shoulderMotor.getPIDController();
  private final RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, ArmConstants.ticksPerRev);

  public ArmSubsystem() {
    // Restore factory defaults in case anything went wrong beforehand
    shoulderMotor.restoreFactoryDefaults();

    shoulderController.setFeedbackDevice(shoulderEncoder);
    shoulderEncoder.setPosition(0);

    elbowMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    elbowMotor.setSelectedSensorPosition(0);
    elbowMotor.setSensorPhase(true);

    shoulderMotor.setIdleMode(IdleMode.kBrake);
    elbowMotor.setNeutralMode(NeutralMode.Brake);

    elbowMotor.configPeakCurrentLimit(35, 10);
    elbowMotor.configPeakCurrentDuration(200, 10);
    elbowMotor.configContinuousCurrentLimit(20, 10);
    elbowMotor.enableCurrentLimit(true);

    shoulderMotor.setSmartCurrentLimit(35);

    shoulderMotor2.setInverted(false);
    shoulderMotor2.follow(shoulderMotor, true);
  }

  public void setShoulderSetpoint(double shoulderPos) {
    shoulderController.setReference(shoulderPos, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elbow encoder ticks: ", getElbowEncoder());
    SmartDashboard.putNumber("shoulder encoder ticks: ", getShoulderEncoder());
    // Put it in smart dashboard
    SmartDashboard.putNumber("Elbow Angle: ", getElbowAngle());
  }

  public double getElbowEncoder() {
    return elbowMotor.getSelectedSensorPosition();
  }

  public double getElbowAngle() {
    return getElbowEncoder() * ArmConstants.elbowTickAngleConv;
  }

  public double getShoulderEncoder() {
    return shoulderEncoder.getPosition();
  }

  public void setEncoders(double shoulderTicks, double elbowTicks) {
    shoulderEncoder.setPosition(shoulderTicks);
    elbowMotor.setSelectedSensorPosition(elbowTicks);
  }

  public void setShoulderPID(double[] pidValues) {
    shoulderController.setP(pidValues[0]);
    shoulderController.setI(pidValues[1]);
    shoulderController.setD(pidValues[2]);
  }
  
  public static double calcThetaOne(
      double endpointX,
      double endpointY,
      double firstArmLength,
      double secondArmLength,
      double thetaTwo) {
    double beta = Math.atan(round(endpointY / endpointX, 4));
    double alpha = Math.atan(round(
        (secondArmLength * Math.sin(thetaTwo))
            / (firstArmLength + secondArmLength * Math.cos(thetaTwo)),
        4));

    return beta - alpha;
  }

  public static double calcGamma(
      double endpointX,
      double endpointY,
      double firstArmLength,
      double secondArmLength) {
    double gamma = round(Math.acos((Math.pow(firstArmLength, 2)
        + Math.pow(secondArmLength, 2)
        - Math.pow(endpointX, 2)
        - Math.pow(endpointY, 2))
        / (2 * firstArmLength * secondArmLength)), 4);
    return gamma;
  }

  public static double round(double number, int decimalPrecision) {
    double factor = Math.pow(10, decimalPrecision);
    return Math.round(number * factor) / factor;
  }

  public void setArm(double endpointX, double endpointY) {
    double gamma = calcGamma(
        endpointX,
        endpointY + ArmConstants.clawClearance,
        ArmConstants.firstArmLength,
        ArmConstants.secondArmLength) * 180 / Math.PI;
    double theta1 = calcThetaOne(
        endpointX,
        endpointY + ArmConstants.clawClearance,
        ArmConstants.firstArmLength,
        ArmConstants.secondArmLength,
        Math.PI - gamma) * 180 / Math.PI;

    double shoulderPos = ArmConstants.ticksPerRev * 70 * theta1 / 360;
    double elbowPos = ArmConstants.elbowTicksPerRev * gamma / 360;

    System.out.println(
      "Shoulder Position: "
      + Double.toString(shoulderPos)
      + " Elbow Position: "
      + Double.toString(elbowPos)
      + Double.toString(shoulderEncoder.getPosition())
    );

    //setEncoderSetpoints(shoulderPos, elbowPos);
  }

  public void teleopShoulder(double speed) {
    shoulderMotor.set(speed*0.5);
    SmartDashboard.putNumber("shoulder power 2 ", speed);
  }

  public void teleopElbow(double speed) {
    elbowMotor.set(speed*0.5);
    SmartDashboard.putNumber("elbow power 2 ", speed);
  }
}
