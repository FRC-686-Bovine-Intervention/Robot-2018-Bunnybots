package frc.robot; 

import frc.robot.lib.joystick.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID;

public class Outtake
{
    public Talon outtakeMotor; 
    public int port; 
    public static double outtakeSpeed = 1;
    public static double outtakeStop = 0; 


    public Outtake(int port)
    {
      outtakeMotor = new Talon(port);
    }

    public void run (boolean btnIsPushed) //possible change - pass in buttons on remote instead of boolean
    {
        if (btnIsPushed == true) {
            outtakeMotor.set(outtakeSpeed);
        } else {
            outtakeMotor.set(outtakeStop);
        }

    }
}