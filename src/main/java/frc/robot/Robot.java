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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.kauailabs.navx.frc.AHRS;

import org.ejml.simple.SimpleSparseOperations;

import frc.robot.SwerveAngle;

import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;


  

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  TalonSRX one = new TalonSRX(1);
  public static TalonSRX two = new TalonSRX(2);
  public static TalonSRX three = new TalonSRX(3);
  TalonSRX four = new TalonSRX(4);
  TalonFX falcRight = new TalonFX(5);
  TalonFX falcLeft = new TalonFX(6);
  TalonFX shooter1 = new TalonFX(7);
  TalonFX shooter2 = new TalonFX(8);
  VictorSPX feeder = new VictorSPX(9);

  
  
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
  public double distancefromtape = 23.5;

  private PIDController pidLimeX = new PIDController(0.04, 0.0, 0.0);
  private PIDController pidLimeY = new PIDController(0.02, 0.00, 0.0);
  public boolean readytoshoot = false;

  double stickLength;

//color stuff start
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
 * A Rev Color Match object is used to register and detect known colors. This can 
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
                 //3 inches away, light on: (0.22,0.453,0.343)
                 //3 inches away, light off:(0.28,0.463,0.232)
                 //6 inches away, light on:(0.256,0.476,0.265)
                 //6 inches away, light off:(0..31,0.464,0.235)
private final Color kRedTarget =  new Color (0.455, 0.4, 0.1);
                 //3 inches away, light on: (0.385,0.425,0.191)
                 //3 inches away, light off:(0.309,0.466,0.221)
                 //6 inches away, light on:(0.249,0.475,0.271)
                 //6 inches away, light off:(0.30,0.465,0.229)     
  public boolean ball_in_low;
  public boolean ball_in_high;
  public boolean intake;
  public boolean shoot;
  public boolean feederboolean;
  public boolean feederSmartDash;
  public boolean readyToShoot;
  TalonSRX falcIntake = new TalonSRX(1);
  TalonSRX falcFeeder = new TalonSRX(2);
  TalonSRX falcShooter1 = new TalonSRX(3);
  TalonSRX falcShooter2 = new TalonSRX(4);
  Timer timer1 = new Timer();
  Timer timer2 = new Timer();
  public double intakeMotor;
  public double shootMotor;
  public double feederMotor;
  public double shooter1SmartDash;
  public double shooter2SmartDash;

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double xlime = tx.getDouble(0.0);
    double ylime = ty.getDouble(0.0);
    double vlime = tv.getDouble(0.0);
    SmartDashboard.putNumber("limelightx", xlime);
    SmartDashboard.putNumber("limelighty", ylime);
    SmartDashboard.putNumber("valid target", xlime);
    SmartDashboard.putBoolean("ready to shoot?", readytoshoot);
    SmartDashboard.putNumber("error pid lime x", pidLimeX.getPositionError());
    SmartDashboard.putNumber("error pid lime y", pidLimeY.getPositionError());

    if(vlime == 1){
      falcLeft.set(TalonFXControlMode.PercentOutput, (pidLimeY.calculate(ylime,distancefromtape)*0.4)+ (pidLimeX.calculate(xlime,0)*-0.4));
      falcRight.set(TalonFXControlMode.PercentOutput, (pidLimeY.calculate(ylime,distancefromtape)*0.4) + (pidLimeX.calculate(xlime,0)*0.4));
    } else{
      falcLeft.set(TalonFXControlMode.PercentOutput, 0);
      falcRight.set(TalonFXControlMode.PercentOutput, 0);
    }
    if(pidLimeY.getPositionError() <= 7.2 && pidLimeY.getPositionError() >= -7.2 && pidLimeX.getPositionError() <= 4.0 && pidLimeX.getPositionError() >= -4.0) {
      readytoshoot = true;
    }else{
      readytoshoot = false;
    }
    }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //two.set(TalonSRXControlMode.MotionMagic, (0 + (angleWrapTimes * 360) * 2.8444444444444));
    //three.set(TalonSRXControlMode.MotionMagic, (0 + (angleWrapTimes * 360) * 2.8444444444444));
    
    //two.set(TalonSRXControlMode., 1024 * (two.getSelectedSensorPosition()/1024));

    lastAngle = 0;
    currentAngle = 0;
    angleWrapTimes = 0;
    currentAngleWrapped = 0;

    navX.zeroYaw();

    

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double xlime = tx.getDouble(0.0);
    double ylime = ty.getDouble(0.0);
    double alime = ta.getDouble(0.0);
    double vlime = tv.getDouble(0.0);

   SmartDashboard.putBoolean("pressedLeftStick", pressedLeftStick);
    if(pressedLeftStick=false){
      steadyHeading = navX.getYaw();
      SmartDashboard.putNumber("steadyHeading", steadyHeading);
      SmartDashboard.putNumber("navXgetYaw", navX.getYaw());
    }

    stickLength = Math.sqrt((gamePad.getLeftX() * gamePad.getLeftX()) + (gamePad.getLeftY() * gamePad.getLeftY()));

    if(gamePad.getLeftX() >= 0.1 || gamePad.getLeftX() <= -0.1 ||  gamePad.getLeftY() >=0.1 || gamePad.getLeftY() <=-0.1) {
      //SwerveAngle.navTog = true;

      SmartDashboard.putNumber("Degrees", Math.toDegrees(Math.atan2(-gamePad.getLeftY(), gamePad.getLeftX()))+180.0);
      SmartDashboard.putNumber("Left X", gamePad.getLeftX());
      SmartDashboard.putNumber("Left Y", gamePad.getLeftY());
      SmartDashboard.putNumber("Right X", gamePad.getRightX());
      SmartDashboard.putNumber("Right Y", gamePad.getRightY());

      SmartDashboard.putNumber("Encoder Two", two.getSelectedSensorPosition());
      SmartDashboard.putNumber("Encoder Three", three.getSelectedSensorPosition());
      
      swerveAngleLeft.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
      swerveAngleRight.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);

      //if((gamePad.getRightX() >= 0.1 || gamePad.getRightX() <= -0.1) /*&& (gamePad.getLeftX() < 0.1 && gamePad.getLeftX() > -0.1 &&  gamePad.getLeftY() <0.1 && gamePad.getLeftY() >-0.1) */  ) { //Test if Right stick is being touched and the Left stick is not being touched
        
      
      SmartDashboard.putNumber("left postion", swerveAngleLeft.currentStickAngle + 180);
      SmartDashboard.putNumber("right postion", swerveAngleRight.currentStickAngle + 180);
      //rotate

        SmartDashboard.putNumber("Multiplier", SwerveAngle.multiplier);
        SmartDashboard.putNumber("three.getSelectedSensorPosition()",three.getSelectedSensorPosition() );
        SmartDashboard.putNumber("swerveAngleRight.angleWrapTimes", swerveAngleRight.angleWrapTimes);
        SmartDashboard.putNumber("three.getSelectedSensorPosition()-swerveAngleRight.angleWrapTimes*1024)", (three.getSelectedSensorPosition())-swerveAngleRight.angleWrapTimes*1024);
        SmartDashboard.putNumber("Error value", Math.abs(three.getClosedLoopError()));
        SmartDashboard.putNumber("stick power", stickLength);
        if(Math.abs(three.getClosedLoopError()) < 50){
         if((three.getSelectedSensorPosition()-swerveAngleRight.angleWrapTimes*1024) < 0) {
           falcRight.set(TalonFXControlMode.PercentOutput, (stickLength * ((gamePad.getRightTriggerAxis() * 0.2) + 0.3)) - /*Math.abs(SwerveAngle.multiplier) * */ (gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
           falcLeft.set(TalonFXControlMode.PercentOutput, (stickLength * ((gamePad.getRightTriggerAxis() * 0.2) + 0.3)) + /*Math.abs(SwerveAngle.multiplier) * */(gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
          } else {
           falcRight.set(TalonFXControlMode.PercentOutput, (stickLength * ((gamePad.getRightTriggerAxis() * 0.2) + 0.3)) + /*Math.abs(SwerveAngle.multiplier) * */(gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
           falcLeft.set(TalonFXControlMode.PercentOutput, (stickLength * ((gamePad.getRightTriggerAxis() * 0.2) + 0.3)) - /* Math.abs(SwerveAngle.multiplier) * */(gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
           } 
        } else {
          falcLeft.set(TalonFXControlMode.PercentOutput, 0);
          falcRight.set(TalonFXControlMode.PercentOutput, 0);
        }
        //}
      swerveAngleRight.rotateCalc(swerveAngleLeft);
      swerveAngleRight.rotateCalc(swerveAngleRight);
      
//if(gamePad.getRightY() >= 0.1 || gamePad.getRightY() <= -0.1) {
      //swerveAngleRight.calc(gamePad.getRightX(), gamePad.getRightY(), three, navX);
//      Math.sqrt((gamePad.getLeftX()*gamePad.getLeftX())+(gamePad.getLeftY()*gamePad.getLeftY()));
//      double rightSpeed = Math.sqrt((gamePad.getRightX()*gamePad.getRightX())+(gamePad.getRightY()*gamePad.getRightY()));
//      }
pressedLeftStick = true;  
  }
    else {
      //SwerveAngle.navTog = false;
      two.set(ControlMode.MotionMagic,1024*swerveAngleLeft.angleWrapTimes - 256);
      three.set(ControlMode.MotionMagic,1024*swerveAngleLeft.angleWrapTimes - 256);
//      swerveAngleLeft.calc(1024*angleWrapTimes, -1, two);
//      swerveAngleRight.calc(1024*angleWrapTimes, -1, three);
      falcRight.set(TalonFXControlMode.PercentOutput, -gamePad.getRightX() * 0.2); 
      falcLeft.set(TalonFXControlMode.PercentOutput,  gamePad.getRightX() * 0.2);

      pressedLeftStick=false;
    }
    if(gamePad.getAButtonPressed()) {
      SwerveAngle.navTog = !SwerveAngle.navTog;
    } 
    SmartDashboard.putBoolean("Field Oriented Control?", SwerveAngle.navTog);

     if (gamePad.getXButton()) {
      navX.zeroYaw();
    }
    SmartDashboard.putBoolean("Shooting", shooting);
    if (gamePad.getYButtonPressed()){
      shootTimer.reset();
      shootTimer.start();
      shooting = true;
    }

    if (shooting && shootTimer.get()<0.5){
      shooter1.set(TalonFXControlMode.PercentOutput, 0.5);
      shooter2.set(TalonFXControlMode.PercentOutput, 0.5);
   } else if (shooting && shootTimer.get()<0.8){
    shooter1.set(TalonFXControlMode.PercentOutput, 0.5);
    shooter2.set(TalonFXControlMode.PercentOutput, 0.5);
    feeder.set(ControlMode.PercentOutput, 0.4);
   } else if (shooting){
      shooting = false;
    }

     //color stuff start
     SmartDashboard.getNumber("inakeMotor", intakeMotor);
     SmartDashboard.getNumber("feederMotor", feederMotor);
     SmartDashboard.getNumber("feederMotor", shootMotor);
     SmartDashboard.getNumber("feederMotor", shooter2SmartDash);
     SmartDashboard.getNumber("feederMotor", shooter1SmartDash);
     if(gamePad.getBButtonPressed()){intake=!intake;}
     if(gamePad.getAButtonPressed()){shoot=!shoot;}
     if(m_colorSensor1.getProximity() > 200.0){ball_in_low=true;} else ball_in_low=false;
     if(m_colorSensor2.getProximity() > 200.0){ball_in_high=true;} else ball_in_high=false;
     if(intake){
      if(ball_in_low = false){
        //falcIntake.set(TalonSRXControlMode.PercentOutput, 0.5);
        intakeMotor = 0.5;
       } else {
        intake=false;
        System.out.println("Ball in low"); // vibrate controller?
        feederboolean=true;
        intakeMotor = 0.0;
       }
     }
     if(feederboolean){
      if(ball_in_low && ball_in_high == false){
       /*falcIntake.set(TalonSRXControlMode.PercentOutput, 0.5);
       falcFeeder.set(TalonSRXControlMode.PercentOutput, 0.5);*/
       intakeMotor = 0.5;
       feederMotor = 0.5;
      }
      if(ball_in_low == false && ball_in_high == false){
       //falcFeeder.set(TalonSRXControlMode.PercentOutput, 0.5);
       feederMotor = 0.5;
      }
      if(ball_in_high){
       System.out.println("Ball in high"); // vibrate controller? 
       feederboolean =false;
       feederMotor = 0.0;
      }
     }
     if(shoot){
      if(vlime == 1){
        falcLeft.set(TalonFXControlMode.PercentOutput, (pidLimeY.calculate(ylime,distancefromtape)*0.4)+ (pidLimeX.calculate(xlime,0)*-0.4));
        falcRight.set(TalonFXControlMode.PercentOutput, (pidLimeY.calculate(ylime,distancefromtape)*0.4) + (pidLimeX.calculate(xlime,0)*0.4));
      }

      if(pidLimeY.getPositionError() <= 7.2 && pidLimeY.getPositionError() >= -7.2 && pidLimeX.getPositionError() <= 4.0 && pidLimeX.getPositionError() >= -4.0) {
        readytoshoot = true;
      }else{
        readytoshoot = false;
      }

      if(ball_in_high){ //timed sequence goes below here
       timer1.reset();
       timer1.start();
       if(timer1.get()<0.5){
        /*falcShooter1.set(TalonSRXControlMode.PercentOutput, 0.5);
        falcShooter2.set(TalonSRXControlMode.PercentOutput, 0.5);*/
         shooter1SmartDash = 0.5;
         shooter2SmartDash = 0.5;
       } else if (readyToShoot=false) {
        /*falcShooter1.set(TalonSRXControlMode.PercentOutput, 0.5);
        falcShooter2.set(TalonSRXControlMode.PercentOutput, 0.5);*/
        shooter1SmartDash = 0.5;
        shooter2SmartDash = 0.5;
        timer2.reset();
        timer2.start();
       } else if (timer2.get()<0.5){
        /*falcShooter1.set(TalonSRXControlMode.PercentOutput, 0.5);
        falcShooter2.set(TalonSRXControlMode.PercentOutput, 0.5);
        falcFeeder.set(TalonSRXControlMode.PercentOutput, 0.5);*/
        shooter1SmartDash = 0.5;
        shooter2SmartDash = 0.5;
        feederMotor = 0.0;
       } else{shoot=false;} // ?and print something to terminal or dashboard?
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if(gamePad.getAButton()) {
      one.setSelectedSensorPosition(1024.0/4.0);
    two.setSelectedSensorPosition(1024.0/4.0);
    three.setSelectedSensorPosition(1024.0/4.0);
    four.setSelectedSensorPosition(1024.0/4.0);
    navX.zeroYaw();
    }    
    swerveAngleLeft.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
    swerveAngleRight.calc(gamePad.getLeftX(), gamePad.getLeftY(), navX);
    
    if(gamePad.getLeftBumperPressed()) { 
      two.setSelectedSensorPosition(-1024.0/4.0);
    }
    if(gamePad.getRightBumperPressed()) {
      three.setSelectedSensorPosition(-1024.0/4.0);
    }

  }

  //Calculates the speed to drive the talons at
  public double driveSpeed(double x, double y) { //inputs must be gamePad.get{left or right}x/y();
    if((y >= 0.15 || x >= 0.15) || (y <= -0.15 || x <= -0.15)) { //sets dead zone
      return Math.sqrt((x*x)+(y*y)) * 0.3; //pythagorean theorem, how far the stick is from center
    }else {
      return 0.0;
    }
  }
  //Caclulates where to turn the swerve
  public void swerveAngle(double x, double y, TalonSRX motor) { //inputs must be gamePad.get{left or right}x/y();
    currentAngle = Math.toDegrees(Math.atan2(y , -x));
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
    motor.set(TalonSRXControlMode.MotionMagic, currentAngleWrapped * (1024.0/360.0));
    lastAngle = currentAngle;

  }

  public static boolean touchingLeftStick() {
    if (gamePad.getLeftX() >= 0.1 || gamePad.getLeftX() <= -0.1 ||  gamePad.getLeftY() >=0.1 || gamePad.getLeftY() <=-0.1) {
      return false;
    } else {
      return true;
    }
  }
  
}




