package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;

public class Climb {

    TalonFX one = new TalonFX(8);
    TalonFX two = new TalonFX(9);

    Servo three = new Servo(4);
    Servo four = new Servo(5);

    public Climb() {
        three.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        four.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    }
    

    public void climb() {
        three.setSpeed(1.0); // to open
        three.setSpeed(-1.0); // to close

        extendMotors();
        retractMotors();


    }

    public void extendMotors() {
        one.set(TalonFXControlMode.Position, 100);
        two.set(TalonFXControlMode.Position, 100);
    }
    public void retractMotors() {
        one.set(TalonFXControlMode.Position, 0);
        two.set(TalonFXControlMode.Position, 0);
    }
}
