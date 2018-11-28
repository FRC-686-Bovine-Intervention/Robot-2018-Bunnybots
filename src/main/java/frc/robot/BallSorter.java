package  frc.robot;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class BallSorter
{
    public static BallSorter mInstance = new BallSorter();
    public static BallSorter getInstance() { return mInstance; }
    public Talon sorterMotor; 
    public final int port = 4; 
    public static double sorterSpeed = 1;
    public static double sorterStop = 0; 


    public BallSorter()
    {
      sorterMotor = new Talon(port);
    }

 
    public void run()
    {/*
        if (event.color)
        {
          Serial.println(event.color); Serial.println("detected");
          sorterMotor.set(sorterSpeed);
        }
        else
        {
          Serial.println("Sensor overload");
          sorterMotor.set(sorterStop);
        }
    */}
}
