package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

public class SwerveAngle {

    double lastAngle;
    double currentAngle;
    double angleWrapTimes;
    double currentAngleWrapped;

    double lastAngleNav;
    double currentAngleNav;
    double angleWrapTimesNav;
    double currentAngleWrappedNav;


    
    public void calc(double x, double y, TalonSRX motor, AHRS navX) { //inputs must be gamePad.get{left or right}x/y();
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
    
    lastAngle = currentAngle;

    currentAngleNav = -navX.getYaw();
    SmartDashboard.putNumber("Current Angle", currentAngle);
    if (lastAngleNav > 90 && currentAngleNav < -90) {
      angleWrapTimesNav++;
    }
    if (lastAngleNav < -90 && currentAngleNav > 90) {
      angleWrapTimesNav--;
    }
    currentAngleWrappedNav = currentAngleNav + angleWrapTimesNav * 360;
    SmartDashboard.putNumber("current nav angle wrapped" + motor.getDeviceID(), currentAngleWrappedNav);
    
    lastAngleNav = currentAngleNav;







    motor.set(TalonSRXControlMode.MotionMagic, (currentAngleWrapped - currentAngleWrappedNav) * (1024.0/360.0));

  }
    
}
