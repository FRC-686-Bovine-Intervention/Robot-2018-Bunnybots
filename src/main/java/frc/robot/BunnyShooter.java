package  frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class BunnyShooter {
    public static BunnyShooter mInstance = new BunnyShooter();
    public static BunnyShooter getInstance() { return mInstance; }
    public DoubleSolenoid bunnySolenoid;
    public final int fwdPort = 5;
    public final int rvsPort = 6;

    public BunnyShooter()
    {
     bunnySolenoid = new DoubleSolenoid(5,6);
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

