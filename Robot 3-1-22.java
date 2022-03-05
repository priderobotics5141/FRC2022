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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

import org.ejml.simple.SimpleSparseOperations;

import frc.robot.SwerveAngle;


  

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
  TalonSRX falcRight = new TalonSRX(5);
  TalonSRX falcLeft = new TalonSRX(6);
  

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

  AHRS navX = new AHRS();

  SwerveAngle swerveAngleLeft = new SwerveAngle(two);
  SwerveAngle swerveAngleRight = new SwerveAngle(three);

  

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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //two.set(TalonSRXControlMode.MotionMagic, (0 + (angleWrapTimes * 360) * 2.8444444444444));
    //three.set(TalonSRXControlMode.MotionMagic, (0 + (angleWrapTimes * 360) * 2.8444444444444));
    

    lastAngle = 0;
    currentAngle = 0;
    angleWrapTimes = 0;
    currentAngleWrapped = 0;

    navX.zeroYaw();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   SmartDashboard.putBoolean("pressedLeftStick", pressedLeftStick);
    if(pressedLeftStick=false){
      steadyHeading = navX.getYaw();
      SmartDashboard.putNumber("steadyHeading", steadyHeading);
      SmartDashboard.putNumber("navXgetYaw", navX.getYaw());
    }

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
        if((three.getSelectedSensorPosition()-swerveAngleRight.angleWrapTimes*1024) < 0) {
          falcRight.set(TalonSRXControlMode.PercentOutput, (gamePad.getRightTriggerAxis() * 0.4) - /*Math.abs(SwerveAngle.multiplier) * */ (gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
          falcLeft.set(TalonSRXControlMode.PercentOutput, (gamePad.getRightTriggerAxis() * 0.4) + /*Math.abs(SwerveAngle.multiplier) * */(gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
        } else {
          falcRight.set(TalonSRXControlMode.PercentOutput, (gamePad.getRightTriggerAxis() * 0.4) + /*Math.abs(SwerveAngle.multiplier) * */(gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
          falcLeft.set(TalonSRXControlMode.PercentOutput, (gamePad.getRightTriggerAxis() * 0.4) - /* Math.abs(SwerveAngle.multiplier) * */(gamePad.getRightX() * 0.1 * (1-Math.abs(SwerveAngle.multiplier))));
        
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
      falcRight.set(TalonSRXControlMode.PercentOutput, -gamePad.getRightX() * 0.2); 
      falcLeft.set(TalonSRXControlMode.PercentOutput,  gamePad.getRightX() * 0.2);

      pressedLeftStick=false;
    }
    if(gamePad.getAButtonPressed()) {
      SwerveAngle.navTog = !SwerveAngle.navTog;
    } 
    SmartDashboard.putBoolean("Field Oriented Control?", SwerveAngle.navTog);

    if(gamePad.getLeftBumperPressed()) { 
      two.setSelectedSensorPosition(-1024.0/4.0);
    }
    if(gamePad.getRightBumperPressed()) {
      three.setSelectedSensorPosition(-1024.0/4.0);
    }
     if (gamePad.getXButton()) {
      navX.zeroYaw();
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




