/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  // Controllers
  private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  // Drivetrain 
  private CANSparkMax leftMaster = new CANSparkMax(Constants.DRIVE_LEFT_MASTER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax leftSlave = new CANSparkMax(Constants.DRIVE_LEFT_SLAVE_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightMaster = new CANSparkMax(Constants.DRIVE_RIGHT_MASTER_CAN_ID, MotorType.kBrushless);
  private CANSparkMax rightSlave = new CANSparkMax(Constants.DRIVE_RIGHT_SLAVE_CAN_ID, MotorType.kBrushless);
  private CANEncoder leftEnc = leftMaster.getEncoder();
  private CANEncoder rightEnc = rightMaster.getEncoder();
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);
  private DifferentialDriveOdometry odometry;
  private boolean isReversed = false;
  private double maxOutput = 1.0;

  // Intake

  // Shooter

  // Pneumatics
  private DoubleSolenoid intakeCyls = new DoubleSolenoid(Constants.INTAKE_CYLINDERS_FORWARD_PORT, Constants.INTAKE_CYLINDERS_REVERSE_PORT);
  private DoubleSolenoid tensionerCyls = new DoubleSolenoid(Constants.TENSIONER_CYLINDERS_FORWARD_PORT, Constants.TENSIONER_CYLINDERS_REVERSE_PORT);

  // Vision
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private double tx, ty, tv;

  // LEDs
  private AddressableLED led = new AddressableLED(Constants.LED_PWM_PORT);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_COUNT);
  private String ledState = "disabled";

  // Auto
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    driveInit();
    visionInit();


    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
    DriverStation.reportWarning("In disabledInit", false);
  }

  @Override
  public void disabledPeriodic() {
    DriverStation.reportWarning("In disabledPeriodic", false);
    if ((isXAligned() && isYAligned()) && ledState.equals("disabled")) {
      ledAligned();
      ledState = "aligned";
    } else if ((isXAligned() || isYAligned()) && ledState.equals("aligned")) {
      ledDisabled();
      ledState = "disabled";
    }
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    ledAutoRunning();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    DriverStation.reportWarning("In teleopInit", false);
    ledTeleop();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }










  // Drivetrain Methods
  public void driveInit() {
    leftMaster.restoreFactoryDefaults();
    leftSlave.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    rightSlave.restoreFactoryDefaults();

    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);

    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftEnc.setPositionConversionFactor(0.4); 
    rightEnc.setPositionConversionFactor(0.4);

    resetDriveEncoders();
    resetGyro();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void drive(double throttle, double yaw) {
    if (isReversed()) {
      leftMaster.set(maxOutput * (throttle+yaw));
      rightMaster.set(maxOutput * (throttle-yaw));

    } else {
      leftMaster.set(maxOutput * (-throttle+yaw)); //-throttle+yaw
      rightMaster.set(maxOutput * (-throttle-yaw)); //-throttle-yaw
    }
    
  }

  public boolean isReversed() {
    return isReversed;
  }

  public void toggleReversed() {
    isReversed = !isReversed;
  }

  public void resetDriveEncoders() {
    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);
  }

  public void resetGyro() {
    navx.zeroYaw();
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }











  // Vision Methods
  public void visionInit() {
    setLimelightLED(1.0);
    setLimelightCamMode(0.0);
  }

  public void updateLimelightValues() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = limelight.getEntry("tv").getDouble(0.0);
  }

  public void setLimelightLED(double state) {
    // 1 = force off, 2 = force blink, 3 = force on
    limelight.getEntry("ledMode").setNumber(state);
  }

  public void setLimelightCamMode(double mode) {
    // 0 = vision processor, 1 = Driver Camera
    limelight.getEntry("camMode").setNumber(mode);
  }

  public boolean isXAligned() {
    return Math.abs(tx) < Constants.LIMELIGHT_YAW_THRESHOLD;
  }

  public boolean isYAligned() {
    return Math.abs(ty) < Constants.LIMELIGHT_THROTTLE_THRESHOLD;
  }

  public boolean hasTarget() {
    return tv == 1.0;
  }








  // LED methods
  public void ledInit() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    ledDisabled();
  }

  public void ledDisabled() {
    for (int i = 0; i < ledBuffer.getLength(); i = i + 2) { // pattern: blue and gold alternating
      ledBuffer.setRGB(i, 212, 175, 55);
      ledBuffer.setRGB(i+1, 0, 0, 102);
    }
    led.setData(ledBuffer);
  }

  public void ledAutoRunning() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: solid blue
      ledBuffer.setRGB(i, 0, 0, 255);
    }
    led.setData(ledBuffer);
  }

  public void ledTeleop() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: white
      ledBuffer.setRGB(i, 255, 255, 255);
    }
    led.setData(ledBuffer);
  }

  public void ledAligned() {
    for (int i = 0; i < ledBuffer.getLength(); i++) { // pattern: solid green
      ledBuffer.setRGB(i, 0, 255, 0);
    }
    led.setData(ledBuffer);
  }






}
