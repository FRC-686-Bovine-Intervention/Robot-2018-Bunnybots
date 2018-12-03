package  frc.robot;

import frc.robot.lib.joystick.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID;
//Intake - 1 Hopper - 2 Color Sorter - 3 Valve - 4 Bouttake - 5 Gouttake - 6

public class Hopper
{
        public static Hopper mInstance = new Hopper();
        public static Hopper getInstance() { return mInstance; }
        public Talon hopperMotor;
        public final int port = 2; 
        public static double hopperSpeed = 1;
        public static double hopperStop = 0;

        public Hopper ()
        {
            hopperMotor = new Talon(port);
        }

        public void run (boolean btnIsPushed) //possible change - pass in buttons on remote instead of boolean - have always running stop button
        {
            JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
            if (btnIsPushed == true) 
            {
                hopperMotor.set(hopperSpeed);
            }
            else if (controls.getButton(Constants.kXboxButtonX));
            {
                hopperMotor.set(hopperStop);
            }
                
        }
}