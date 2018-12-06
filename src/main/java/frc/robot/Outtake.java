package frc.robot;

import edu.wpi.first.wpilibj.Talon;

public class Outtake
{
    public static final int goodOuttakePort = 5;
    public static final int badOuttakePort = 6;
  
    public static Outtake mGoodInstance = new Outtake(goodOuttakePort);
    public static Outtake getGoodInstance() { return mGoodInstance; }
    public static Outtake mBadInstance = new Outtake(badOuttakePort);
    public static Outtake getBadInstance() { return mBadInstance; }

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
            start();
        } else {
           done();
        }

    }

    public void start()
    {
        outtakeMotor.set(outtakeSpeed);
    }
    public void done()
    {
        outtakeMotor.set(outtakeStop);
    }
}