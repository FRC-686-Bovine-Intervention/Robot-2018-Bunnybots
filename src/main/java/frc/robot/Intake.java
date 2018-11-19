package  frc.robot;

import frc.robot.lib.joystick.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID;

public class Intake
{
    public static Intake mInstance = new Intake();
    public static Intake getInstance() { return mInstance; }
    public Talon intakeMotor; 
    public final int port = 3; 
    public static double intakeSpeed = 1;
    public static double intakeStop = 0; 


    public Intake()
    {
      intakeMotor = new Talon(port);
    }


    public void run()
    {
      JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
      if (controls.getButton(Constants.kXboxButtonLB)) {
        intakeMotor.set(intakeSpeed);
      } else {
        intakeMotor.set(intakeStop);
      }
    }
}
