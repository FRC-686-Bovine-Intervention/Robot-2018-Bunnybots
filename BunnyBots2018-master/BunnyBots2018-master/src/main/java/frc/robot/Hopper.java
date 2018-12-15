package  frc.robot;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class Hopper {
        public static Hopper mInstance = new Hopper();
        public static Hopper getInstance() { return mInstance; }
        public Talon hopperMotor;
        public final int port = 2; 
        public static double hopperSpeed = 0.50;
        public static double hopperStop = 0;

        public Hopper ()
        {
            hopperMotor = new Talon(port);
        }

        public void run (boolean btnIsPushed) //possible change - pass in buttons on remote instead of boolean - have always running stop button
        {
            hopperMotor.set(hopperSpeed);   
        }
}