package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;

public class Climb {

    TalonFX fxOne = new TalonFX(11);
    TalonFX fxTwo = new TalonFX(12);

    Servo servo1 = new Servo(5);
    //Servo servo2 = new Servo(6);

    public Climb() { //init
        servo1.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        //servo2.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    }
    

    public void start() {
        if(Robot.climbBool) {
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
        servo1.setSpeed(-1.0);
        //four.setSpeed(-1.0);
    }
    public void servoMid() {
        servo1.setSpeed(0);
        //four.setSpeed(1.0);
    }
    public void servoForward() {
        servo1.setSpeed(1.0);
    }

}
