package frc.robot.auto.modes;

import java.util.Arrays;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.GoodOuttakeAction;
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.DriveLoop;

/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class DriveStraightIntakeMode extends AutoModeBase {

    public DriveStraightIntakeMode() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Center Start Mode");
       // PathSegment.Options pathOptions	= new PathSegment.Options(DriveLoop.kPathFollowingMaxVel, DriveLoop.kPathFollowingMaxAccel, 48, false);
        //PathSegment.Options tightTurnOptions	= new PathSegment.Options(DriveLoop.kPathFollowingMaxVel/2, DriveLoop.kPathFollowingMaxAccel, 24, false);
        //Vector2d backupPosition = 		new Vector2d(24, 0);
		//Vector2d cubePickupPosition = 	new Vector2d(152 - 3*13 - Constants.kCenterToExtendedIntake +  0, 0);	
        //Vector2d driveToCenterPosition = new Vector2d(0,835.0);
        //Vector2d stopDrivePosition = new Vector2d(0,0);
       // Vector2d driveToCratePosition = new Vector2d(0,0);
       // Vector2d cratePosition = new Vector2d(0,0);
       // Vector2d startPosition = new Vector2d(0,0);

        /*
        Path TurnPath = new Path();
		TurnPath.add(new Waypoint(startPosition, tightTurnOptions));	
        TurnPath.add(new Waypoint(driveToCenterPosition, tightTurnOptions));
        TurnPath.setReverseDirection();
        */
        /*
        Path sharpTurnPath = new Path();	
        sharpTurnPath.add(new Waypoint(turnPosition, pathOptions));
        sharpTurnPath.add(new Waypoint(driveToBallsStartPosition, pathOptions));
        sharpTurnPath.setReverseDirection();
        */
/*
        Path driveToCenterPath = new Path();
		driveToCenterPath.add(new Waypoint(driveToCenterPosition, pathOptions));	
        driveToCenterPath.add(new Waypoint(stopDrivePosition, pathOptions));
*/
       // Path ballIntakePath = new Path();
		//ballIntakePath.add(new Waypoint(stopDrivePosition, pathOptions));	
        //ballIntakePath.add(new Waypoint(driveToCratePosition, pathOptions));

       // Path backupToStartPath = new Path();
		//backupToStartPath.add(new Waypoint(driveToCratePosition, pathOptions));	
        //backupToStartPath.add(new Waypoint(startPosition, pathOptions));
        /*
        Path turnToCratePath = new Path();
		turnToCratePath.add(new Waypoint(turnToCratePosition, pathOptions));	
        turnToCratePath.add(new Waypoint(driveToCratePosition, pathOptions));
        */
        /*
        Path driveToCratePath = new Path();
		driveToCratePath.add(new Waypoint(driveToCratePosition, tightTurnOptions));	
        driveToCratePath.add(new Waypoint(cratePosition, pathOptions));

        Path turnAroundPath = new Path();
		turnAroundPath.add(new Waypoint(cratePosition, tightTurnOptions));	
        turnAroundPath.add(new Waypoint(outtakeStartPosition, tightTurnOptions));
        */

        runAction(new GoodOuttakeAction()); 
        runAction(new WaitAction(1.0));
        runAction(new BunnyBotDriveAction());
       // runAction(new ParallelAction (Arrays.asList(new Action[] {
           // (new PathFollowerAction(ballIntakePath)),
          //  (new IntakeAction())})));    
        
      //  runAction(new PathFollowerAction(backupToStartPath));
        }
    }