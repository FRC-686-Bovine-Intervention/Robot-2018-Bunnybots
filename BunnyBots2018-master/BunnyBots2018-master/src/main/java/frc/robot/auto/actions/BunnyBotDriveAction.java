package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.command_status.DriveState;
import frc.robot.subsystems.*;

/**
 * BunnyBotDriveAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class BunnyBotDriveAction implements Action {

    private Drive drive = Drive.getInstance();

    public BunnyBotDriveAction() {
       
    }


    @Override
    public void start() 
    {
        double speed = 0.25;
        DriveCommand driveCmd = new DriveCommand(speed, speed);
        drive.setOpenLoop(driveCmd);
    }

    @Override
    public void update() 
    {
    }

    @Override
    public boolean isFinished() 
    {
    }

    @Override
    public void done()
    {
        drive.setOpenLoop(DriveCommand.COAST());
    }

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
