package frc.robot;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.networktables.*;

//use solenoids to program phnematics on valve
public class ColorSorter
{
    public static ColorSorter mInstance = new ColorSorter();
    public static ColorSorter getInstance() { return mInstance; }
    public final int port = 4;
    public Jaguar colorSortMotor;
    boolean redDetected = false;
    public final double speed = +1.0;

    public ColorSorter() 
    {
     colorSortMotor = new Jaguar(port);
    }

    public void run ()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = table.getEntry("tv");
        double v = tv.getDouble(0);
            if (v > 0.5) {
            redDetected = true;
            }
            else
            {
            redDetected = false;
            }

            if (redDetected) {

            colorSortMotor.set(+speed);
    
            } else {
            colorSortMotor.set(-speed);
            }
    }


}

