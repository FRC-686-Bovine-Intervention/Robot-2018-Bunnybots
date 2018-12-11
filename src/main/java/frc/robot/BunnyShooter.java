package  frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class BunnyShooter {
    public static BunnyShooter mInstance = new BunnyShooter();
    public static BunnyShooter getInstance() { return mInstance; }
    public DoubleSolenoid bunnySolenoid;
    public DoubleSolenoid gateSolenoid;
    public final int gFwdPort = 1;
    public final int gRvsPort = 2;
    public final int bFwdPort = 5;
    public final int bRvsPort = 6;

    public BunnyShooter()
    {
     bunnySolenoid = new DoubleSolenoid(5,6);
     gateSolenoid = new DoubleSolenoid(1,2);
    }


    public void run()
    {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        if (controls.getButton(Constants.kXboxButtonY)) {
            gateSolenoid.set(DoubleSolenoid.Value.kForward);
          bunnySolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
          bunnySolenoid.set(DoubleSolenoid.Value.kReverse);
          gateSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }
}

