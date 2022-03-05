/* To-Do List:
  *Rotate while moving sideways





*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;

public class SwerveAngle {

    double lastStickAngle;
    double currentStickAngle;
    double angleWrapTimes;
    double currentStickAngleWrapped;

    double lastAngleNav;
    double currentAngleNav;
    double angleWrapTimesNav;
    double currentAngleWrappedNav;

    double multiplier = Math.cos((2.0*Math.PI/1024.0)*((Robot.three.getSelectedSensorPosition())));

    double maximumSwerveAnglePivotWhileDrivingModifyer = 45;

    TalonSRX motor;
    

    public static boolean navTog = true;


    public SwerveAngle(TalonSRX motor) {
      this.motor = motor;
    }
    
    public void calc(double x, double y, AHRS navX) { //inputs must be gamePad.get{left or right}x/y();
    currentStickAngle = Math.toDegrees(Math.atan2(y , -x));
    SmartDashboard.putNumber("Current Angle", currentStickAngle);
    multiplier = Math.cos((2.0*Math.PI/1024.0)*((Robot.three.getSelectedSensorPosition())));
    SmartDashboard.putNumber("multiplier", multiplier);
    if (lastStickAngle > 90 && currentStickAngle < -90) {
      angleWrapTimes++;
    }
    if (lastStickAngle < -90 && currentStickAngle > 90) {
      angleWrapTimes--;
    }
    currentStickAngleWrapped = currentStickAngle + angleWrapTimes * 360;
    SmartDashboard.putNumber("navx", navX.getYaw());
    SmartDashboard.putNumber("current angle wrapped" + motor.getDeviceID(), currentStickAngleWrapped);
    
    lastStickAngle = currentStickAngle;
    if(navTog) {
      currentAngleNav = -navX.getYaw();
      SmartDashboard.putNumber("Current Angle", currentStickAngle);
      if (lastAngleNav > 90 && currentAngleNav < -90) {
        angleWrapTimesNav++;
      }
      if (lastAngleNav < -90 && currentAngleNav > 90) {
        angleWrapTimesNav--;
      }
      currentAngleWrappedNav = currentAngleNav + angleWrapTimesNav * 360;
      SmartDashboard.putNumber("current nav angle wrapped" + motor.getDeviceID(), currentAngleWrappedNav);
      
      lastAngleNav = currentAngleNav;
      double multiplier = Math.cos((2.0*Math.PI/1024.0)*((Robot.three.getSelectedSensorPosition())));

      motor.set(TalonSRXControlMode.MotionMagic, (currentStickAngleWrapped - currentAngleWrappedNav) * (1024.0/360.0));
    } else if (Robot.gamePad.getRightX() >= 0.1 || Robot.gamePad.getRightX() <= -0.1 && Robot.touchingLeftStick()) {   
      if (false/*motor.getDeviceID() == 3*/) {
        motor.set(TalonSRXControlMode.MotionMagic, (currentStickAngleWrapped + Robot.gamePad.getRightX()*maximumSwerveAnglePivotWhileDrivingModifyer) * (1024.0/360.0)+(motor.getSelectedSensorPosition()+512.0)%512.0); //Janzen says: add another multiplier here that checks how close you are to pointing bot-sideways
      } else {
        motor.set(TalonSRXControlMode.MotionMagic, (currentStickAngleWrapped + Robot.gamePad.getRightX()*-maximumSwerveAnglePivotWhileDrivingModifyer) * (1024.0/360.0)*multiplier*100);
      }
    } else {
      motor.set(TalonSRXControlMode.MotionMagic, currentStickAngleWrapped * (1024.0/360.0));
    }
  }

  public void calc(double x, double y, TalonSRX motor) {
    
    currentStickAngle = Math.toDegrees(Math.atan2(y , -x));
    SmartDashboard.putNumber("Current Angle", currentStickAngle);
    multiplier = Math.cos((2.0*Math.PI/1024.0)*((Robot.three.getSelectedSensorPosition())));
    SmartDashboard.putNumber("Multiplier", multiplier);
    if (lastStickAngle > 90 && currentStickAngle < -90) {
      angleWrapTimes++;
    }
    if (lastStickAngle < -90 && currentStickAngle > 90) {
      angleWrapTimes--;
    }
    currentStickAngleWrapped = currentStickAngle + angleWrapTimes * 360;
    SmartDashboard.putNumber("current angle wrapped" + motor.getDeviceID(), currentStickAngleWrapped);
    
    lastStickAngle = currentStickAngle;

    currentStickAngleWrapped = currentStickAngle + angleWrapTimes * 360; //WHY ARE WE DOING THIS TWICE??

    
    double multiplier = Math.cos((2.0*Math.PI/1024.0)*((Robot.three.getSelectedSensorPosition())));

    motor.set(TalonSRXControlMode.MotionMagic, currentStickAngleWrapped * (1024.0/360.0));

  }


  public void rotateCalc(SwerveAngle side) {
    /*multiplierLeft = -(((((Robot.two.getSelectedSensorPosition() + 180.0) - 180.0) * (1.0 - 0.0)) / (270.0 - 180.0)) + 0.0);
    if(multiplierLeft > 1.0){
      multiplierLeft
 = 1.0;
    } else if (multiplierLeft < -1.0) {
      multiplierLeft
 = -1.0;
    }
    SmartDashboard.putNumber("Left Multiplyer", multiplierLeft);

    multiplyerRight = (((((Robot.two.getSelectedSensorPosition() + 180.0) - 0.0) * (1.0 - 0.0)) / (90.0 - 0.0)) + 0.0);
    if(multiplyerRight > 1.0){
      multiplyerRight = 1.0;
    } else if (multiplyerRight < -1.0) {
      multiplyerRight = -1.0;
    }
    SmartDashboard.putNumber("Right Multiplyer", multiplyerRight);
*/

    


    

  }

    
}
