package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.kauailabs.navx.frc.AHRS;

public class DrivetrainSubsystem extends SubsystemBase {

  // Make motors
  private final WPI_TalonFX frontLeft = new WPI_TalonFX(DrivetrainConstants.frontLeftPort);
  private final WPI_TalonFX backLeft = new WPI_TalonFX(DrivetrainConstants.backLeftPort);
  private final WPI_TalonFX frontRight = new WPI_TalonFX(DrivetrainConstants.frontRightPort);
  private final WPI_TalonFX backRight = new WPI_TalonFX(DrivetrainConstants.backRightPort);
  private final MecanumDrive mecDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

  //Make the gyro
  private final AHRS gyro = new AHRS(Port.kUSB);

  //Smart dashboard
  private NetworkTable limelightTable;
  private NetworkTableEntry txEntry;
  private NetworkTableEntry tyEntry;
  private NetworkTableEntry tvEntry;
  private NetworkTableEntry taEntry;
  private NetworkTableEntry ledModeEntry;

  public DrivetrainSubsystem() {
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    
    //Set the encoder
    frontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    //Reset the gyro
    gyro.reset();
    gyro.calibrate();
    
    frontLeft.setSelectedSensorPosition(0);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void resetGyro(){
    gyro.reset();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }
  public void setBrakeMode(){
    frontLeft.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode(){
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);
  }

  public void resetEncoderTicks() {
    frontLeft.setSelectedSensorPosition(0);
  }

  //make mecanumdrive robot oriented instead of field-oriented
  public void setMecanumRobotOriented(double xSpeed, double ySpeed, double rotationSpeed){
    mecDrive.driveCartesian(xSpeed, ySpeed, rotationSpeed);
  }

  public void setMecanum(double xSpeed, double ySpeed, double rotationSpeed){
    mecDrive.driveCartesian(xSpeed, ySpeed, rotationSpeed, Rotation2d.fromDegrees(gyro.getAngle()));
  }

  //Encoder Value get
  public double getEncoderValue(){
    return frontLeft.getSelectedSensorPosition();
  }

  // Get velocity
  public double getVelocity(){
    return frontLeft.getSelectedSensorVelocity();
  }

  public double getAngle(){
   return gyro.getAngle();
  }

  public double getPitch(){
    return gyro.getPitch();
  }

  //Limelight, get x angle offset
  public double getTX(){
    return txEntry.getDouble(0);
  }

  //Get limelight y angle offset
  public double getTY(){
    return tyEntry.getDouble(0);
  }

  //Is there a reflective tape in our vision? Get it (returns 1 if yes, 0 if no)

  public double getTA(){
    return taEntry.getDouble(0);
  }

  //Every 20 ms, update the smartdashboard values
  @Override
  public void periodic() {
    tyEntry = limelightTable.getEntry("ty");
    txEntry = limelightTable.getEntry("tx");
    taEntry = limelightTable.getEntry("ta");
    tvEntry = limelightTable.getEntry("tv");

    SmartDashboard.putNumber("tv", tvEntry.getDouble(-1));
    SmartDashboard.putNumber("Encoder Ticks", getEncoderValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
