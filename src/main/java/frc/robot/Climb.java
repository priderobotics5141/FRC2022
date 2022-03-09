package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;

public class Climb {

    TalonFX fxOne = new TalonFX(8);
    TalonFX fxTwo = new TalonFX(9);

    Servo three = new Servo(4);
    Servo four = new Servo(5);

    public Climb() { //init
        three.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        four.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    }
    

    public void climb() {
        extendMotors();
        retractMotors();
        servoBack();
        extendMotors();
        servoMid();
        retractMotors();
        servoBack();
        extendMotors();
        retractMotors();
        servoBack();
        extendMotors();
        servoMid();
        retractMotors();

    }

    public void extendMotors() {
        fxOne.set(TalonFXControlMode.Position, 100);
        fxTwo.set(TalonFXControlMode.Position, 100);
    }
    public void retractMotors() {
        fxOne.set(TalonFXControlMode.Position, 0);
        fxTwo.set(TalonFXControlMode.Position, 0);
    }

    public void servoBack() {
        three.setSpeed(-1.0);
        four.setSpeed(-1.0);
    }
    public void servoMid() {
        three.setSpeed(1.0);
        four.setSpeed(1.0);
    }

}
