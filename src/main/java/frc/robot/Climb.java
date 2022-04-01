package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;

public class Climb {

    public TalonFX fx1 = new TalonFX(11);
    public TalonFX fx2 = new TalonFX(12);

    Servo servo1 = new Servo(5);
    Servo servo2 = new Servo(7);

    Timer climbTime = new Timer();

    public boolean climbMode;

    public Climb() { //init
        servo1.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        servo2.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    }
    

    public void start() {
        if(Robot.climbBool) {

           // ArrayList times = new ArrayList<Double>;

           if(climbTime.get() > 0 && climbTime.get() <= 0) {
               extendMotors();
           }
           if(climbTime.get() >= 1 && climbTime.get() <= 1.02) {

           }

            /*climbTime.schedule(extendMotors, 0);
            climbTime.schedule(retractMotors, 2000);
            climbTime.schedule(servoLevel, 2300);
            climbTime.schedule(extendMotors, 2800);
            climbTime.schedule(servoBack, 3000);
            climbTime.schedule(servoForward, 5000);
            climbTime.schedule(retractMotors, 5300);
            climbTime.schedule(servoForward, 5400);
            climbTime.schedule(servoLevel, 5700);*?

            /*extendMotors();
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
            retractMotors();*/
        }

    }
    public void manualClimb() {
        if(climbMode) {
            if(Robot.gamePad.getAButton()) {
                servoForward();
              }
              if(Robot.gamePad.getBButton()) {
                servoMid();
              }
              if(Robot.gamePad.getXButton()) {
                servoLevel();
              }
              if(Robot.gamePad.getYButton()) {
                servoBack();
              }
              if(Robot.gamePad.getRightTriggerAxis() > 0.0) 
              {
                fx1.set(ControlMode.PercentOutput, Robot.gamePad.getRightTriggerAxis() * 0.4);
                fx2.set(ControlMode.PercentOutput, Robot.gamePad.getRightTriggerAxis() * 0.4);
              } else 
              {
                fx1.set(ControlMode.PercentOutput, Robot.gamePad.getLeftTriggerAxis() * -0.4);
              fx2.set(ControlMode.PercentOutput, Robot.gamePad.getLeftTriggerAxis() * -0.4);
              }
              

        }
    }
    /*TimerTask extendMotors = new TimerTask() {
        public void run() {
            fx1.set(TalonFXControlMode.Position, 100);
            fx2.set(TalonFXControlMode.Position, 100);
        }
    }; TimerTask retractMotors = new TimerTask() {
        public void run() {
            fx1.set(TalonFXControlMode.Position, 0);
            fx2.set(TalonFXControlMode.Position, 0);        
        }
    };
    TimerTask servoBack = new TimerTask() {
        public void run() {
            servo1.setSpeed(1);
            servo2.setSpeed(1);
        }
    }; TimerTask servoMid = new TimerTask() {
        public void run() {
            servo1.setSpeed(0);
            servo2.setSpeed(0);        }
    }; TimerTask servoLevel = new TimerTask() {
        public void run() {
            servo1.setSpeed(-0.2);
            servo2.setSpeed(-0.2);
        }
    }; TimerTask servoForward = new TimerTask() {
        public void run() {
            servo1.setSpeed(-1.0);
            servo2.setSpeed(-1.0);    
        }
    };*/

    public void extendMotors() {
        fx1.set(TalonFXControlMode.Position, 100);
        fx2.set(TalonFXControlMode.Position, 100);
    }
    public void retractMotors() {
        fx1.set(TalonFXControlMode.Position, 0);
        fx2.set(TalonFXControlMode.Position, 0);
    }

    public void setMotors(double value) {
        fx1.set(TalonFXControlMode.Position, value);
        fx2.set(TalonFXControlMode.Position, value);
    }

    public void servoBack() {
        servo1.setSpeed(-1.0);
        servo2.setSpeed(-1.0);
    }
    public void servoLevel() {
        servo1.setSpeed(-0.2);
        servo2.setSpeed(-0.2);
    }
    public void servoMid() {
        servo1.setSpeed(0.0);
        servo2.setSpeed(0.0);
    }
    public void servoForward() {
        servo1.setSpeed(1.0);
        servo2.setSpeed(1.0);
    }


    

}
