package frc.robot;

import edu.wpi.first.wpilibj.Talon;

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

    public void run (boolean btnIsPushed) //possible change - pass in buttons on remote instead of boolean poss. BTN LB god & LB+ btn for bad
    {
        if (btnIsPushed == true) {
            outtakeMotor.set(outtakeSpeed);
        } else {
            outtakeMotor.set(outtakeStop);
        }

    }
}