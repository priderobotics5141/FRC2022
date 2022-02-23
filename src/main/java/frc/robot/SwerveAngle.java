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

    double multiplierLeft;
    double multiplierForward;

    double maximumSwerveAnglePivotWhileDrivingModifyer = 45;
    

    public static boolean navTog;


    
    public void calc(double x, double y, TalonSRX motor, AHRS navX) { //inputs must be gamePad.get{left or right}x/y();
    currentStickAngle = Math.toDegrees(Math.atan2(y , -x));
    SmartDashboard.putNumber("Current Angle", currentStickAngle);
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
      motor.set(TalonSRXControlMode.MotionMagic, (currentStickAngleWrapped - currentAngleWrappedNav) * (1024.0/360.0));
    } else if (Robot.gamePad.getRightX() >= 0.1 || Robot.gamePad.getRightX() <= -0.1 && Robot.touchingLeftStick()) {   
      if (motor.getDeviceID() == 3) {
        motor.set(TalonSRXControlMode.MotionMagic, (currentStickAngleWrapped + Robot.gamePad.getRightX()*maximumSwerveAnglePivotWhileDrivingModifyer) * (1024.0/360.0)+(motor.getSelectedSensorPosition()+512.0)%512.0); //Janzen says: add another multiplier here that checks how close you are to pointing bot-sideways
      } else {
        motor.set(TalonSRXControlMode.MotionMagic, (currentStickAngleWrapped + Robot.gamePad.getRightX()*-maximumSwerveAnglePivotWhileDrivingModifyer) * (1024.0/360.0));
      }
    } else {
      motor.set(TalonSRXControlMode.MotionMagic, currentStickAngleWrapped * (1024.0/360.0));
    }
  }

  public void calc(double x, double y, TalonSRX motor) {
    
    currentStickAngle = Math.toDegrees(Math.atan2(y , -x));
    SmartDashboard.putNumber("Current Angle", currentStickAngle);
    if (lastStickAngle > 90 && currentStickAngle < -90) {
      angleWrapTimes++;
    }
    if (lastStickAngle < -90 && currentStickAngle > 90) {
      angleWrapTimes--;
    }
    currentStickAngleWrapped = currentStickAngle + angleWrapTimes * 360;
    SmartDashboard.putNumber("current angle wrapped" + motor.getDeviceID(), currentStickAngleWrapped);
    
    lastStickAngle = currentStickAngle;

    currentStickAngleWrapped = currentStickAngle + angleWrapTimes * 360;

    motor.set(TalonSRXControlMode.MotionMagic, currentStickAngleWrapped * (1024.0/360.0));

  }


  public void rotateCalc(double rightStick) {
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

    multiplierLeft = (Robot.three.getSelectedSensorPosition()+512.0)%512.0/256.0;
    SmartDashboard.putNumber("Side to side multiplyer", multiplierLeft);
    multiplierForward = (Robot.three.getSelectedSensorPosition()+256.0)%256.0/256.0;
    SmartDashboard.putNumber("Forward and back multiplyer", multiplierForward);

  }
    
}
