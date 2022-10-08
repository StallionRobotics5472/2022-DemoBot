package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Static method containing all constant values for the robot in one location
 */
public final class Constants {


  // Motor IDS
  
  public static final int PLAYER_ONE_PORT = 0;
    public static final int PLAYER_TWO_PORT = 1;
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BACK_LEFT_BUTTON = 5;
    public static final int BACK_RIGHT_BUTTON = 6;
    public static final int START_BUTTON = 8;
    public static final int BACK_BUTTON = 7;
    public static final int X_AXIS_BUTTON = 9;
    public static final int Y_AXIS_BUTTON = 10;
    public static final int JOYSTICK_X_AXIS = 0;
    public static final int JOYSTICK_Y_AXIS = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int ALTERNATE_JOYSTICK_X_AXIS = 4;
    public static final int ALTERNATE_JOYSTICK_Y_AXIS = 5;
    public static final int Ball_Sensor = 1;
  public static int intake = 8;
  public static int rightDropper = 9;
  public static int leftDropper = 10;
  public static int transport = 11;
  public static int bottomFlywheel = 12;
  public static int topFlywheel = 13;
  public static int hood = 14;
  public static int rightclimb = 15;
  public static int leftClimb = 16;
  public static final double wheelDiameterMeters = Units.inchesToMeters(3.1); 
  public static final double driveMotorGearRatio = 1 / 5.25; 
  public static final double angleMotorGearRatio = 1 / 53.3; 
  public static final double driveRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameterMeters; 
  public static final double angleRot2Rad =   Math.PI * 2 * angleMotorGearRatio; 
  public static final double driveRPM2MPS = driveRot2Meter / 60; 
  public static final double angleRPM2RPS = angleRot2Rad / 60; 
  public static final double kPangle = 0.6; 
  public static final double kIangle = 0; 
  public static final double kDangle = 0; 
  public static final double kTranslationSlew = 1.750;
  public static final double kRotationSlew = 3.75;
  //the max drive speed is small just for testing purposes. 
  public static final double MAXDriveSpeed = 8; 
  public static final double kMaxAngularSpeed = 2*Math.PI;
  public static final double autoDriveSpeed = 0.2; 
      public static final double Deadband = 0.2;
      public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                    // read when not in use (Eliminates "Stick Drift")
  public static final double kOuterDeadband = 0.98;
      
      //left and right distance  
      public static final double trackWidth = Units.inchesToMeters(20); 
  //front and back distance
  public static final double wheelBase = Units.inchesToMeters(20);
  //
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2, -trackWidth /2),
      new Translation2d(wheelBase / 2, trackWidth /2), 
      new Translation2d(-wheelBase /2, -trackWidth /2),
      new Translation2d(-wheelBase /2, trackWidth /2)); 

  //module constants 
  public static final int frontRight = 3;
  public static final int frontLeft = 0;
  public static final int backRight = 2;
  public static final int backLeft = 1;
  public static final int a_frontRight = 7;
  public static final int a_frontLeft = 4;
  public static final int a_backRight = 6;
  public static final int a_backLeft = 5;

  public static final boolean driveMotorReverse = true; 
  public static final boolean driveMotorForward = false;
  
  public static final boolean angleMotorReverse = true; 
  public static final boolean angleMotorFoward = false; 

  public static final double encoderOffsetRad = 4.376396;

  public static final double[] kKeepAnglePID = { 0.600, 0, 0 };
  public static final double kMinRotationCommand = Constants.kMaxAngularSpeed
  * Math.pow(Constants.kInnerDeadband, 2);
  public static final double kMinTranslationCommand = Constants.MAXDriveSpeed
      * Math.pow(Constants.kInnerDeadband, 2);
}
