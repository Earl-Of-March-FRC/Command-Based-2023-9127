package frc.robot;

public final class Constants {
  public static class LEDConstants{
    public static final int PWMPort = 0;
    public static final int stripLength = 58;
  }
  public static class OperatorConstants {
    public static final int controllerPort = 0;
    public static final int forwardAxis = 1;
    public static final int sideAxis = 0;
    public static final int rotationAxis = 2;
  }

  public static class ArmConstants {
    public static final int shoulderPort = 8;
    public static final int shoulderPort2 = 7;
    public static final int elbowPort = 6;
    public static final int clawPort = 9;
    
    public static final double firstArmLength = 24;
    public static final double secondArmLength = 20;
    public static final double clawClearance = 0;
    public static final int ticksPerRev = 42;
    public static final int elbowTicksPerRev = 4096;
    public static final double elbowTickAngleConv = 360.0/elbowTicksPerRev;

    // Hybrid, low cube, low cone, high cube, high cone
    public static final double[] targetsX = {8, 22.75, 22.75, 39.75, 39.75};
    public static final double[] targetsY = {0, 16, 16, 16, 46};

    // setpoints
    public static final double ELBOW_STOWED = 1300;
    public static final double SHOULDER_STOWED = -10;
    public static final double ELBOW_PICKUP_SETPOINT = 500;
    public static final double SHOULDER_PICKUP_SETPOINT = -45;
    public static final double ELBOW_LOW_CUBE_SETPOINT = 1052;
    public static final double SHOULDER_LOW_CUBE_SETPOINT = -105;

    // feedforward
    public static final double ELBOW_PICKUP_FEEDFORWARD = 0.25;
    public static final double SHOULDER_PICKUP_FEEDFORWARD = 0;
    public static final double ELBOW_LOW_CUBE_FEEDFORWARD = 0.2;
    public static final double SHOULDER_LOW_CUBE_FEEDFORWARD = 0.0078;

    // pid values
    public static final double[] ELBOW_STOWED_PID = {0.012, 0, 0};
    public static final double[] SHOULDER_STOWED_PID = {0.012, 0, 0};
    public static final double[] ELBOW_PICKUP_PID = {0.0075, 0, 0};
    public static final double[] SHOULDER_PICKUP_PID = {0.0125, 0, 0};
    public static final double[] ELBOW_LOW_CUBE_PID = {0.014, 0, 0};
    public static final double[] SHOULDER_LOW_CUBE_PID = {0.05, 0, 0};
  }

  public static class ClawConstants {
    public static final double CLAW_SPIN_IN = -0.5;
    public static final double CLAW_SPIN_OUT = 0.8;
  }

  public static class DrivetrainConstants {
    public static final int frontLeftPort = 1; 
    public static final int backLeftPort = 4;
    public static final int frontRightPort = 2;
    public static final int backRightPort = 3;

    public static final double ticksPerRev = 2048*12.71;
    public static final double inchesPerRev = Math.PI * 3.0 * 2;


    //driving speed
    public static final double forwardScaleFactor = 0.8;
    public static final double sidewaysScaleFactor = 0.8;
    public static final double rotateScaleFactor = 0.4;

    public static final double forwardDeadzone = 0.1;
    public static final double sidewaysDeadzone = 0.1;
    public static final double rotateDeadzone = 0.1;
  
    public static class NormalPID {
      public static final double kP = 0.0002;
      public static final double kI = 0.000000085;
      public static final double kD = 0.000004;
      public static final double kS = 0;
      public static final double kV = 0;
      public static final double kA = 0;
    }

    public static class RotationPID {
      public static final double kP = 0.0013;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }

  public static class IntakeConstants {
    public static final int intakePort = 10;
    public static final int tripperPort = 5;
  }
}
