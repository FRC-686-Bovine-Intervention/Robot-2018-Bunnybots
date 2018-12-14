package  frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.Constants;

public class BunnyShooter {
    public static BunnyShooter mInstance = new BunnyShooter();
    public static BunnyShooter getInstance() { return mInstance; }
    public DoubleSolenoid bunnySolenoid;
    public final int bFwdPort = 5;
    public final int bRvsPort = 6;

    public BunnyShooter()
    {
     bunnySolenoid = new DoubleSolenoid(0, bFwdPort, bRvsPort);
    }


    public void run()
    {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        if (controls.getButton(Constants.kXboxButtonY)) {
            bunnySolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
          bunnySolenoid.set(DoubleSolenoid.Value.kReverse);
         }
    }
}

