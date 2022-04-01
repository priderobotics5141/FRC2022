// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* To-Do List:
  * FRC PID
  * Prevent passive roation
  * translate forward while rotating right should rotate the same direction as translate backward while rotating right
  *
  *
*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.NetworkInterface;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.kauailabs.navx.frc.AHRS;

import org.ejml.simple.SimpleSparseOperations;

import frc.robot.SwerveAngle;
import frc.robot.Climb;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // TalonSRX one = new TalonSRX(1);
  public static TalonSRX two = new TalonSRX(2);
  public static TalonSRX three = new TalonSRX(3);
  // TalonSRX four = new TalonSRX(4);
  TalonFX falcRight = new TalonFX(6); // switched from 5 to 6, 3/20/22
  TalonFX falcLeft = new TalonFX(5); // switched from 6 to 5, 3/20/22

  TalonFX falcShooter1 = new TalonFX(7);
  TalonFX falcShooter2 = new TalonFX(8);
  VictorSPX feederMotor = new VictorSPX(9);
  VictorSPX intakeMotor = new VictorSPX(10);
  TalonFX climb1 = new TalonFX(11);
  TalonFX climb2 = new TalonFX(12);

  Servo test = new Servo(6);

  Climb climb = new Climb();

  public static XboxController gamePad = new XboxController(0);

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  double lastAngle;
  double currentAngle;
  double angleWrapTimes;
  double currentAngleWrapped;
  boolean pressedLeftStick = false;
  double steadyHeading;
  boolean shooting;

  Timer shootTimer = new Timer();

  AHRS navX = new AHRS();

  SwerveAngle swerveAngleLeft = new SwerveAngle(two);
  SwerveAngle swerveAngleRight = new SwerveAngle(three);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  public double distancefromtape = 11.0;

  private PIDController pidLimeX = new PIDController(0.04, 0.0, 0.0);
  private PIDController pidLimeY = new PIDController(0.02, 0.0005, 0.0);
  private PIDController pidFalcLeftDistance = new PIDController(0.00002, 0.0, 0.0);
  private PIDController pidFalcRightDistance = new PIDController(0.00002, 0.0, 0.0);

  public boolean readytoshoot = false;
  public static boolean climbBool = false;

  double throttle;

  double rawAngle;
  double tempAngle;
  double trueAngle;
  double trueError;
  double stickDegree;

  double a = 90;
  double b;

  private final I2C.Port i2cPort1 = I2C.Port.kOnboard;
  private final I2C.Port i2cPort2 = I2C.Port.kMXP;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a
   * parameter. The device will be automatically initialized with default
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor1 = new ColorSensorV3(i2cPort1);
  private final ColorSensorV3 m_colorSensor2 = new ColorSensorV3(i2cPort2);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher1 = new ColorMatch();
  private final ColorMatch m_colorMatcher2 = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.156, 0.406, 0.432);
  /*
   * 3 inches away, light on: (0.22,0.453,0.343)
   * //3 inches away, light off:(0.28,0.463,0.232)
   * //6 inches away, light on:(0.256,0.476,0.265)
   * //6 inches away, light off:(0..31,0.464,0.235)
   */
  private final Color kRedTarget = new Color(0.455, 0.4, 0.1);
  /*
   * 3 inches away, light on: (0.385,0.425,0.191)
   * //3 inches away, light off:(0.309,0.466,0.221)
   * //6 inches away, light on:(0.249,0.475,0.271)
   * //6 inches away, light off:(0.30,0.465,0.229)
   */
  public boolean ball_in_low;
  public boolean ball_in_high;
  public boolean intake;
  public boolean shoot;
  public boolean feederBool;
  public boolean feederSmartDash;
  public boolean readyToShoot;
  Timer timer1 = new Timer();
  Timer timer2 = new Timer();
  public double intakeValue;
  public double shootMotor;
  public double feederValue;
  public double shooterSpeed;
  public double shooter2SmartDash;
  public double wheeloffsetleft;
  public double wheeloffsetright;

  public double servoTestSpeed = 0.0;
  private boolean limeDrive;
  private boolean intakeToggle;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    falcShooter2.setInverted(true); // furiosa only
    climb2.setInverted(true); // furiosa only
    // three.setInverted(true);//furiosa only

    two.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    three.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
    two.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10);
    three.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 10);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    falcRight.setSelectedSensorPosition(0);
    falcLeft.setSelectedSensorPosition(0);

    navX.zeroYaw();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //if (m_colorSensor2.getProximity() > 130.0) {
    //  ball_in_high = true;
    //} else {
    //  ball_in_high = false;
    //}
    double xlime = tx.getDouble(0.0);
    double ylime = ty.getDouble(0.0);
    double vlime = tv.getDouble(0.0);
    SmartDashboard.putNumber("limelightx", xlime);
    SmartDashboard.putNumber("limelighty", ylime);
    SmartDashboard.putNumber("valid target", xlime);
    SmartDashboard.putBoolean("ready to shoot?", readytoshoot);
    SmartDashboard.putNumber("error pid lime x", pidLimeX.getPositionError());
    SmartDashboard.putNumber("error pid lime y", pidLimeY.getPositionError());

    /*
     * if(vlime == 1){
     * falcLeft.set(TalonFXControlMode.PercentOutput,
     * (pidLimeY.calculate(ylime,distancefromtape)*0.4)+
     * (pidLimeX.calculate(xlime,0)*-0.4));
     * falcRight.set(TalonFXControlMode.PercentOutput,
     * (pidLimeY.calculate(ylime,distancefromtape)*0.4) +
     * (pidLimeX.calculate(xlime,0)*0.4));
     * } else{
     * falcLeft.set(TalonFXControlMode.PercentOutput, 0);
     * falcRight.set(TalonFXControlMode.PercentOutput, 0);
     * }
     * if(pidLimeY.getPositionError() <= 7.2 && pidLimeY.getPositionError() >= -7.2
     * && pidLimeX.getPositionError() <= 4.0 && pidLimeX.getPositionError() >= -4.0)
     * {
     * readytoshoot = true;
     * }else{
     * readytoshoot = false;
     * }
     */

    SmartDashboard.putNumber("error pid falc", pidFalcLeftDistance.getPositionError());
    //if (pidFalcLeftDistance.getPositionError() <= 3000) {
      /*
       * if(vlime == 1 && !readytoshoot){
       * falcLeft.set(TalonFXControlMode.PercentOutput,
       * (pidLimeY.calculate(ylime,distancefromtape)*0.4)+
       * (pidLimeX.calculate(xlime,0)*-0.4));
       * falcRight.set(TalonFXControlMode.PercentOutput,
       * (pidLimeY.calculate(ylime,distancefromtape)*0.4) +
       * (pidLimeX.calculate(xlime,0)*0.4));
       * } //else{shoot=false;}
       * 
       * if(pidLimeY.getPositionError() <= 8.2 && pidLimeY.getPositionError() >= -8.2
       * && pidLimeX.getPositionError() <= 4.0 && pidLimeX.getPositionError() >= -4.0)
       * {
       * readytoshoot = true;
       * }else{
       * readytoshoot = false;
       * }
       */
      // 11 5 inc = -12.22
      System.out.println("if one");
      readytoshoot = true;
      //if (ball_in_high) { // timed sequence goes below here
        // timer2.start();
        timer1.start();
        System.out.println("if two");
        if (timer1.get() < 0.6) {
          System.out.println("if three");
          shooterSpeed = 0.4;
          falcShooter1.set(TalonFXControlMode.PercentOutput, shooterSpeed);
          falcShooter2.set(TalonFXControlMode.PercentOutput, shooterSpeed);
        } else if (timer1.get() < 2.0) {
          System.out.println("if four");
          shooterSpeed = 0.4;
          falcShooter1.set(TalonFXControlMode.PercentOutput, shooterSpeed);
          falcShooter2.set(TalonFXControlMode.PercentOutput, shooterSpeed);
          feederValue = 0.5;
          feederMotor.set(VictorSPXControlMode.PercentOutput, feederValue);/*
           * timer2.reset();
           * timer2.stop();
           */
        } else if (timer1.get() < 3.0){
          falcShooter1.set(TalonFXControlMode.PercentOutput, 0.0);
          falcShooter2.set(TalonFXControlMode.PercentOutput, 0.0);
          feederMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
        } else  /* ball_in_low? timer2.get()<2.0: timer2.get()<1.0 */  {
          System.out.println("if five");
          
          falcLeft.set(ControlMode.PercentOutput,
          pidFalcLeftDistance.calculate(falcLeft.getSelectedSensorPosition(), -150000) * 0.7); // 36 inches * (10000/9)=40000
          falcRight.set(ControlMode.PercentOutput,
          pidFalcRightDistance.calculate(falcRight.getSelectedSensorPosition(), -150000) * 0.7);
          /*
           * shooterSpeed = 0.5;
           * shooter2SmartDash = 0.5;
           */
        } /* else {
          System.out.println("else");
          
           * shooterSpeed = 0.0;
           * shooter2SmartDash = 0.0;
           * feederValue = 0.0;
           
          timer1.stop();
          timer2.stop();
          timer2.reset();
          timer1.reset();
          shoot = false;
          readytoshoot = false;
          
        }*/ // ?and print something to terminal or dashboard?
      //} else 
      //{
        
      //}

    //}

    // falcLeft.set(TalonFXControlMode.Position, 300);
    // falcRight.set(TalonFXControlMode.Position, 300);
    SmartDashboard.putNumber("falcRight.getSelectedSensorPosition()", falcRight.getSelectedSensorPosition());


    // Dante
    /*
     * rawAngle = two.getSelectedSensorPosition()*360/1024;
     * if (rawAngle >= 0 ){tempAngle = rawAngle % 360;}
     * else{tempAngle = 360 - (rawAngle%360);}
     * 
     * if (tempAngle <= 180){trueAngle = tempAngle;}
     * else{trueAngle = -360 + tempAngle;}
     * 
     * if(Math.abs(trueAngle - a) <= 180){ trueError = trueAngle - a;}
     * else{trueError = trueAngle - a - 360*Math.signum(trueAngle - a);}
     * 
     * SmartDashboard.putNumber("Bit Position", two.getSelectedSensorPosition());
     * SmartDashboard.putNumber("tempAngle", tempAngle);
     * SmartDashboard.putNumber("trueAngle", trueAngle);
     * SmartDashboard.putNumber("trueError", trueError);
     * 
     * SmartDashboard.putNumber("degree", gamePad.getLeftY());
     * SmartDashboard.putNumber("stickdegree", stickDegree);
     * 
     * if (gamePad.getLeftX() >= 0 && (-gamePad.getLeftY()) >= 0){ b =
     * 360/(2*Math.PI)*Math.atan(gamePad.getLeftX()/(-gamePad.getLeftY()));}
     * if (gamePad.getLeftX() < 0 && (-gamePad.getLeftY()) >= 0){ b = 90
     * -360/(2*Math.PI)*Math.atan(gamePad.getLeftX()/(-gamePad.getLeftY()));}
     * if (gamePad.getLeftX() < 0 && (-gamePad.getLeftY()) < 0){ b = 180 +
     * 360/(2*Math.PI)*Math.atan(gamePad.getLeftX()/(-gamePad.getLeftY()));}
     * if (gamePad.getLeftX() >= 0 && (-gamePad.getLeftY()) < 0){ b = 270
     * -360/(2*Math.PI)*Math.atan(gamePad.getLeftX()/(-gamePad.getLeftY()));}
     * 
     * if (b <= 180){stickDegree = b;}
     * else{stickDegree = -360 + b;}
     */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // two.set(TalonSRXControlMode.MotionMagic, (0 + (angleWrapTimes * 360) *
    // 2.8444444444444));
    // three.set(TalonSRXControlMode.MotionMagic, (0 + (angleWrapTimes * 360) *
    // 2.8444444444444));

    // two.set(TalonSRXControlMode., 1024 * (two.getSelectedSensorPosition()/1024));

    lastAngle = 0;
    currentAngle = 0;
    angleWrapTimes = 0;
    currentAngleWrapped = 0;

    test.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    // intakeValue = 0.3;
    // feederValue = 0;
    shootMotor = 0;
    shooter2SmartDash = 0;
    shooterSpeed = 0;

    ball_in_low = false;
    ball_in_high = false;
    intake = false;
    shoot = false;
    feederBool = false;

    timer1.reset();
    timer2.reset();

    three.setSelectedSensorPosition(-1024.0 / 4.0);
    two.setSelectedSensorPosition(-1024.0 / 4.0);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("timer1", timer1.get());
    SmartDashboard.putNumber("timer2", timer2.get());
    SmartDashboard.putBoolean("feederBool", feederBool);
    SmartDashboard.putNumber("error lime y", pidLimeY.getPositionError());
    SmartDashboard.putBoolean("ready to shoot", readytoshoot);
    SmartDashboard.putBoolean("shoot", shoot);
    SmartDashboard.putBoolean("intake", intake);
    SmartDashboard.putNumber("inakeMotor", intakeValue);
    SmartDashboard.putNumber("feederValue", feederValue);
    /*
     * SmartDashboard.putNumber("shoot2Motor", shooter2SmartDash);
     * SmartDashboard.putNumber("shoot1Motor", shooterSpeed);
     */
    SmartDashboard.putBoolean("ball in low", ball_in_low);
    SmartDashboard.putBoolean("ball in high", ball_in_high);
    SmartDashboard.putNumber("proxy 1", m_colorSensor1.getProximity());
    SmartDashboard.putNumber("proxy 2", m_colorSensor2.getProximity());
    SmartDashboard.putBoolean("pressedLeftStick", pressedLeftStick);

    double xlime = tx.getDouble(0.0);
    double ylime = ty.getDouble(0.0);
    double alime = ta.getDouble(0.0);
    double vlime = tv.getDouble(0.0);

    /*if (pressedLeftStick = false) {
      steadyHeading = navX.getYaw();
      SmartDashboard.putNumber("steadyHeading", steadyHeading);
      SmartDashboard.putNumber("navXgetYaw", navX.getYaw());
    }*/

    if (gamePad.getStartButton()) {
      shoot = false;
      intake = false;
      feederBool = false;

      ball_in_low = false;
      ball_in_high = false;
      intakeToggle = false;
    }

    // throttle = Math.sqrt((gamePad.getLeftX() * gamePad.getLeftX()) +
    // (gamePad.getLeftY() * gamePad.getLeftY()));
    throttle = gamePad.getRightTriggerAxis();
    // System.out.println(gamePad.getLeftX() + " " + gamePad.getLeftY());
    if (throttle > 1.0) {
      throttle = 1.0;
    }

    //if (!shoot) {
      // if(gamePad.getLeftX() >= 0.1 || gamePad.getLeftX() <= -0.1 ||
      // gamePad.getLeftY() >=0.1 || gamePad.getLeftY() <=-0.1) {
      // SwerveAngle.navTog = true;

      /*
       * SmartDashboard.putNumber("Degrees",
       * Math.toDegrees(Math.atan2(-gamePad.getLeftY(), gamePad.getLeftX()))+180.0);
       * SmartDashboard.putNumber("Left X", gamePad.getLeftX());
       * SmartDashboard.putNumber("Left Y", gamePad.getLeftY());
       * SmartDashboard.putNumber("Right X", gamePad.getRightX());
       * SmartDashboard.putNumber("Right Y", gamePad.getRightY());
       * 
       * SmartDashboard.putNumber("Encoder Two", two.getSelectedSensorPosition());
       * SmartDashboard.putNumber("Encoder Three", three.getSelectedSensorPosition());
       * 
       */

      // if((gamePad.getRightX() >= 0.1 || gamePad.getRightX() <= -0.1) /*&&
      // (gamePad.getLeftX() < 0.1 && gamePad.getLeftX() > -0.1 && gamePad.getLeftY()
      // <0.1 && gamePad.getLeftY() >-0.1) */ ) { //Test if Right stick is being
      // touched and the Left stick is not being touched

      SmartDashboard.putNumber("left postion", swerveAngleLeft.currentStickAngle + 180);
      SmartDashboard.putNumber("right postion", swerveAngleRight.currentStickAngle + 180);
      // rotate

      SmartDashboard.putNumber("Multiplier", SwerveAngle.multiplier);
      SmartDashboard.putNumber("three.getSelectedSensorPosition()", three.getSelectedSensorPosition());
      SmartDashboard.putNumber("swerveAngleRight.angleWrapTimes", swerveAngleRight.angleWrapTimes);
      SmartDashboard.putNumber("three.getSelectedSensorPosition()-swerveAngleRight.angleWrapTimes*1024)",
          (three.getSelectedSensorPosition()) - swerveAngleRight.angleWrapTimes * 1024);
      SmartDashboard.putNumber("Error value", Math.abs(three.getClosedLoopError()));
      SmartDashboard.putNumber("stick power", throttle);
      if (!climb.climbMode) {
        // System.out.print(Math.abs(two.getClosedLoopError()) + " ");
        // if(Math.abs(two.getClosedLoopError()) < 50.0){
        double m_RightTrigger = gamePad.getRightTriggerAxis();
        if (Math.abs(m_RightTrigger) < 0.1) {
          m_RightTrigger = 0.0;
        }
        double m_TurboMultiplyer = 0.0;

        if (Math.abs(throttle) < 0.1) { // Not moving the left stick. Right stick rotates
          throttle = 0.0;
        } else {
          throttle = throttle * (m_TurboMultiplyer + 0.5);
        }

        double m_RightStick = gamePad.getRightX();
        if (Math.abs(m_RightStick) < 0.1) {
          m_RightStick = 0.0;
        }

        double m_RotateMultiplyer = m_RightStick * 0.1 * (1 - Math.abs(SwerveAngle.multiplier));

        System.out.println("Stick: " + throttle + " Turbo: " + m_TurboMultiplyer + " Right: " + m_RightStick
            + " Rotate: " + m_RotateMultiplyer + " Right Trigger: " + m_RightTrigger);

        if ((three.getSelectedSensorPosition() - swerveAngleRight.angleWrapTimes * 1024) < 0) { //
                                                                                                // System.out.println("Drive
                                                                                                // Wheels Normal");
          SmartDashboard.putNumber("falcon One ",
              (throttle + /* Math.abs(SwerveAngle.multiplier) * */(m_RotateMultiplyer)));
          falcRight.set(TalonFXControlMode.PercentOutput,
              (throttle - /* Math.abs(SwerveAngle.multiplier) * */ (m_RotateMultiplyer)));
          falcLeft.set(TalonFXControlMode.PercentOutput,
              (throttle + /* Math.abs(SwerveAngle.multiplier) * */(m_RotateMultiplyer)));
        } else {
          // System.out.println("Drive wheels inverted");
          falcRight.set(TalonFXControlMode.PercentOutput,
              (throttle + /* Math.abs(SwerveAngle.multiplier) * */(m_RotateMultiplyer)));
          falcLeft.set(TalonFXControlMode.PercentOutput,
              (throttle - /* Math.abs(SwerveAngle.multiplier) * */(m_RotateMultiplyer)));
        }
        if (throttle == 0.0) 
        { // Not touching throttle causes swerve to face forward and right stick allowsrotation
          two.set(ControlMode.MotionMagic, 1024 * swerveAngleLeft.angleWrapTimes - 256);
          three.set(ControlMode.MotionMagic, 1024 * swerveAngleLeft.angleWrapTimes - 256);
          falcRight.set(TalonFXControlMode.PercentOutput, -gamePad.getRightX() * 0.6);
          falcLeft.set(TalonFXControlMode.PercentOutput, gamePad.getRightX() * 0.6);
        } else { // Toching throttle allows movement like normal
          swerveAngleLeft.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
          swerveAngleRight.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
        }
        // } else {
        // System.out.println("Wheels stopped");
        // SmartDashboard.putNumber("falcon One ", 0);
        // falcLeft.set(TalonFXControlMode.PercentOutput, 0);
        // falcRight.set(TalonFXControlMode.PercentOutput, 0);
        // }
      } else { // climb mode movement controll
        swerveAngleLeft.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
        swerveAngleRight.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
        falcRight.set(TalonFXControlMode.PercentOutput, -gamePad.getRightY() * 0.4);
        SmartDashboard.putNumber("climb driver", gamePad.getRightY() * 0.4);
        falcLeft.set(TalonFXControlMode.PercentOutput, -gamePad.getRightY() * 0.4);
      }

      // }
      swerveAngleRight.rotateCalc(swerveAngleLeft);
      swerveAngleRight.rotateCalc(swerveAngleRight);

      // if(gamePad.getRightY() >= 0.1 || gamePad.getRightY() <= -0.1) {
      // swerveAngleRight.calc(gamePad.getRightX(), gamePad.getRightY(), three, navX);
      // Math.sqrt((gamePad.getLeftX()*gamePad.getLeftX())+(gamePad.getLeftY()*gamePad.getLeftY()));
      // double rightSpeed =
      // Math.sqrt((gamePad.getRightX()*gamePad.getRightX())+(gamePad.getRightY()*gamePad.getRightY()));
      // }
      pressedLeftStick = true;
      // }
      // else {
      // SwerveAngle.navTog = false;
      // two.set(ControlMode.MotionMagic,1024*swerveAngleLeft.angleWrapTimes - 256);
      // three.set(ControlMode.MotionMagic,1024*swerveAngleLeft.angleWrapTimes - 256);
      // swerveAngleLeft.calc(1024*angleWrapTimes, -1, two);
      // swerveAngleRight.calc(1024*angleWrapTimes, -1, three);

      pressedLeftStick = false;
      // }
    //}
    /*
     * if(gamePad.getAButtonPressed()) {
     * SwerveAngle.navTog = !SwerveAngle.navTog;
     * }
     */
    SmartDashboard.putBoolean("Field Oriented Control?", SwerveAngle.navTog);

    /*
     * if (gamePad.getXButton()) {
     * navX.zeroYaw();
     * }
     */
    SmartDashboard.putBoolean("Shooting", shooting);
    if (!climb.climbMode) {
      if (gamePad.getYButtonPressed()) {
        shootTimer.reset();
        shootTimer.start();
        shooting = true;
      }
    }
    if (gamePad.getRightBumper()) {
      falcShooter1.set(TalonFXControlMode.PercentOutput, 0.4);
      falcShooter2.set(TalonFXControlMode.PercentOutput, 0.4);
    } else {
      falcShooter1.set(TalonFXControlMode.PercentOutput, 0.0);
      falcShooter2.set(TalonFXControlMode.PercentOutput, 0.0);
    }
    SmartDashboard.putBoolean("intake On", intakeToggle);
    if (gamePad.getLeftBumper()) 
    {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.8);
      feederMotor.set(VictorSPXControlMode.PercentOutput, 0.8);
    } else if (gamePad.getBButton()) 
    {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, -0.7);
      feederMotor.set(VictorSPXControlMode.PercentOutput, -0.7);
    } else
    {
      intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
      feederMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }


    /*
     * if (shooting && shootTimer.get()<0.5){
     * shooter1.set(TalonFXControlMode.PercentOutput, 0.5);
     * shooter2.set(TalonFXControlMode.PercentOutput, 0.5);
     * } else if (shooting && shootTimer.get()<0.8){
     * shooter1.set(TalonFXControlMode.PercentOutput, 0.5);
     * shooter2.set(TalonFXControlMode.PercentOutput, 0.5);
     * feeder.set(ControlMode.PercentOutput, 0.4);
     * } else if (shooting){
     * shooting = false;
     * }
     */

    SmartDashboard.putBoolean("climb", climbBool);
    if (gamePad.getBackButton() && gamePad.getStartButton()) {
      climbBool = true;
      climb.start();
    }
    if (gamePad.getLeftBumper() && gamePad.getRightBumper()) {
      climbBool = false;
    }
    if (!climb.climbMode) {
      if (gamePad.getYButton()) {
        test.setSpeed(1.0);
      }
    }
    /*
     * if(climbBool){
     * //climb sequence goes here
     * climb1.set(TalonFXControlMode.PercentOutput, 0.14);
     * climb2.set(TalonFXControlMode.PercentOutput, 0.14);
     * } else{
     * climb1.set(TalonFXControlMode.PercentOutput, 0.0);
     * climb2.set(TalonFXControlMode.PercentOutput, 0.0);
     * }
     */
    if (!climb.climbMode) {
      if (gamePad.getXButtonPressed()) {
        intake = true;
      }
      if (gamePad.getAButtonPressed()) {
        //shoot = true;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
      }

      if (m_colorSensor1.getProximity() > 175.0) {
        ball_in_low = true;
      } else {
        ball_in_low = false;
      }
      if (m_colorSensor2.getProximity() > 130.0) {
        ball_in_high = true;
      } else {
        ball_in_high = false;
      }
      /*
       * if(intake){
       * if(!ball_in_high){
       * intakeValue = 0.5;
       * feederValue = 0.5;
       * } else if(!ball_in_low){
       * intakeValue = 0.5;
       * feederValue = 0;
       * } else{
       * intakeValue = 0;
       * feederValue = 0;
       * intake = false;
       * }
       * }
       */
      if (intake) {
        if (ball_in_low == false) {
          intakeValue = 1;
          intakeMotor.set(VictorSPXControlMode.PercentOutput, intakeValue);
        } else {
          //feederBool = true;
          intake = false;
          // System.out.println("Ball in low"); // vibrate controller?
          //intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
        }
      } else {
        //intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
      }
      if (/*feederBool*/ false) {
        if (ball_in_low && ball_in_high == false) {
          intakeValue = 1;
          feederValue = 0.5;
          intakeMotor.set(VictorSPXControlMode.PercentOutput, intakeValue);
          feederMotor.set(VictorSPXControlMode.PercentOutput, feederValue);
        }
        if (ball_in_low == false && ball_in_high == false) {
          feederValue = 0.5;
          feederMotor.set(VictorSPXControlMode.PercentOutput, feederValue);
        }
        if (ball_in_high) {
          // System.out.println("Ball in high"); // vibrate controller?
          feederBool = false;
          feederMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
          intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
        }
      } /*
         * else {
         * feederMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
         * }
         */

      //if (shoot) {
        /*
         * if(vlime == 1 && !readytoshoot){
         * falcLeft.set(TalonFXControlMode.PercentOutput,
         * (pidLimeY.calculate(ylime,distancefromtape)*0.4)+
         * (pidLimeX.calculate(xlime,0)*-0.4));
         * falcRight.set(TalonFXControlMode.PercentOutput,
         * (pidLimeY.calculate(ylime,distancefromtape)*0.4) +
         * (pidLimeX.calculate(xlime,0)*0.4));
         * } //else{shoot=false;}
         * 
         * if(pidLimeY.getPositionError() <= 8.2 && pidLimeY.getPositionError() >= -8.2
         * && pidLimeX.getPositionError() <= 4.0 && pidLimeX.getPositionError() >= -4.0)
         * {
         * readytoshoot = true;
         * }else{
         * readytoshoot = false;
         * }
         */

        // 11 5 inc = -12.22

        readytoshoot = true;
        if (/*ball_in_high*/ false) { // timed sequence goes below here
          // timer2.start();
          timer1.start();
          if (timer1.get() < 0.5) {
            shooterSpeed = 0.7;
            falcShooter1.set(TalonFXControlMode.PercentOutput, shooterSpeed);
            falcShooter2.set(TalonFXControlMode.PercentOutput, shooterSpeed);
          } else if (readytoshoot == false) {
            shooterSpeed = 0.7;
            falcShooter1.set(TalonFXControlMode.PercentOutput, shooterSpeed);
            falcShooter2.set(TalonFXControlMode.PercentOutput, shooterSpeed);
            /*
             * timer2.reset();
             * timer2.stop();
             */
          } else if (ball_in_high && readytoshoot/* ball_in_low? timer2.get()<2.0: timer2.get()<1.0 */) {
            falcShooter1.set(TalonFXControlMode.PercentOutput, shooterSpeed);
            falcShooter2.set(TalonFXControlMode.PercentOutput, shooterSpeed);
            feederValue = 0.5;
            feederMotor.set(VictorSPXControlMode.PercentOutput, feederValue);
            /*
             * shooterSpeed = 0.5;
             * shooter2SmartDash = 0.5;
             */

          } else {
            //falcShooter1.set(TalonFXControlMode.PercentOutput, 0.0);
            //falcShooter2.set(TalonFXControlMode.PercentOutput, 0.0);
            feederMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
            /*
             * shooterSpeed = 0.0;
             * shooter2SmartDash = 0.0;
             * feederValue = 0.0;
             */
            timer1.stop();
            timer2.stop();
            timer2.reset();
            timer1.reset();
            shoot = false;
            readytoshoot = false;
          } // ?and print something to terminal or dashboard?
        }
      //} else {
        //falcShooter1.set(TalonFXControlMode.PercentOutput, 0.0);
        //falcShooter2.set(TalonFXControlMode.PercentOutput, 0.0);
      //}
    }
    if (!shoot) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    }
    climb.manualClimb();
    if (gamePad.getBackButtonPressed()) {
      climb.climbMode = !climb.climbMode;
    }
    SmartDashboard.putBoolean("climb Mode", climb.climbMode);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    falcRight.setSelectedSensorPosition(0);
    falcLeft.setSelectedSensorPosition(0);

    servoTestSpeed = -1.0;

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /*
     * if(gamePad.getAButton()) {
     * one.setSelectedSensorPosition(1024.0/4.0);
     * two.setSelectedSensorPosition(1024.0/4.0);
     * three.setSelectedSensorPosition(1024.0/4.0);
     * four.setSelectedSensorPosition(1024.0/4.0);
     * navX.zeroYaw();
     * }
     */
    /*
     * java.util.Timer test5 = new java.util.Timer();
     * TimerTask servoForward = new TimerTask() {
     * public void run() {
     * System.out.println("joe mama");
     * }
     * };
     */
    if (gamePad.getAButton()) {
      climb.servoForward();
    }
    if (gamePad.getBButton()) {
      climb.servoMid();
    }
    if (gamePad.getXButton()) {
      climb.servoLevel();
    }
    if (gamePad.getYButton()) {
      climb.servoBack();
    }

    if (gamePad.getRightBumperPressed()) {
      climb.servo1.setSpeed(servoTestSpeed);
      climb.servo2.setSpeed(servoTestSpeed);
      servoTestSpeed = servoTestSpeed + 0.1;

    }

    // swerveAngleLeft.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
    // swerveAngleRight.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);

    if (gamePad.getLeftBumperPressed()) {
      // two.setSelectedSensorPosition(-1024.0/4.0);
    }
    if (gamePad.getRightBumperPressed()) {
      // three.setSelectedSensorPosition(-1024.0/4.0);
    }

    if (!climb.climbMode) 
    { // throttle
      SmartDashboard.putNumber("distance", falcRight.getSelectedSensorPosition());
      falcRight.set(TalonFXControlMode.PercentOutput, gamePad.getRightTriggerAxis() * .3);
      falcLeft.set(TalonFXControlMode.PercentOutput, gamePad.getRightTriggerAxis() * .3);
    }
  }

  // Calculates the speed to drive the talons at
  public double driveSpeed(double x, double y) { // inputs must be gamePad.get{left or right}x/y();
    if ((y >= 0.15 || x >= 0.15) || (y <= -0.15 || x <= -0.15)) { // sets dead zone
      return Math.sqrt((x * x) + (y * y)) * 0.3; // pythagorean theorem, how far the stick is from center
    } else {
      return 0.0;
    }
  }

  // Caclulates where to turn the swerve
  public void swerveAngle(double x, double y, TalonSRX motor) { // inputs must be gamePad.get{left or right}x/y();
    currentAngle = Math.toDegrees(Math.atan2(y, -x));
    SmartDashboard.putNumber("Current Angle", currentAngle);
    if (lastAngle > 90 && currentAngle < -90) {
      angleWrapTimes++;
    }
    if (lastAngle < -90 && currentAngle > 90) {
      angleWrapTimes--;
    }
    currentAngleWrapped = currentAngle + angleWrapTimes * 360;
    SmartDashboard.putNumber("navx", navX.getYaw());
    SmartDashboard.putNumber("current angle wrapped" + motor.getDeviceID(), currentAngleWrapped);
    motor.set(TalonSRXControlMode.MotionMagic, currentAngleWrapped * (1024.0 / 360.0));
    lastAngle = currentAngle;

  }

  public static boolean touchingLeftStick() {
    if (gamePad.getLeftX() >= 0.1 || gamePad.getLeftX() <= -0.1 || gamePad.getLeftY() >= 0.1
        || gamePad.getLeftY() <= -0.1) {
      return false;
    } else {
      return true;
    }
  }

}
