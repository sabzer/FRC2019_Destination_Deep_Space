package frc.robot.subsystem.autonomous;



import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.drive.DriveConstants;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;



public class AutoAssist extends Command {
    private static OI oi = OI.instance();
    private static AutonomousSubsystem autonomousSubsystem = AutonomousSubsystem.instance();
    private static DriveSubsystem driveSubsystem = DriveSubsystem.instance();

    private GuidanceAlgorithm guidance;



    public AutoAssist() {
        requires(autonomousSubsystem);
        setRunWhenDisabled(true); // Idle command



        guidance = new GuidanceAlgorithm();
    }



    @Override
    protected void initialize() {
        System.out.println(this.getClass().getName() + " AUTO START " + " " + System.currentTimeMillis()/1000);
        autonomousSubsystem.disable();
    }
    
	public static double map(double x, double inMin, double inMax, double outMin, double outMax)
	{
	   return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	}

    @Override
    protected boolean isFinished() {
        driveSubsystem.setVDriveEnable(false);

        boolean forceIdle = oi.driverIdle();

        if (forceIdle) {
            return false;
        }



        boolean useAutoAssist = autonomousSubsystem.getUseAutoAssist();

        if (!useAutoAssist) {
            return CommandUtils.stateChange(new Idle());
        }
        


        CameraFeedback feedback = autonomousSubsystem.getClosestObjectData();

        double turnRate_radps = 0.0;

        if (feedback != null)
        {
            boolean inAutoAssistRegion = feedback.isInAutoAssistRegion();

            // can no longer help
            if (inAutoAssistRegion == false) {
                return CommandUtils.stateChange(new Idle());
            }
 
            double offAxis = feedback.getOffAxis();
            double parallax = feedback.getParallax();
            double distance = feedback.getDistance();

            // no need to keep guiding if in a certain distance
            if (distance <= AutonomousConstants.GUIDANCE_STOP) {
                return CommandUtils.stateChange(new Idle());
            }

            guidance.setOffAxis(offAxis);
            guidance.setParallax(parallax);



            turnRate_radps = guidance.getTurnRate(distance);
        }

        double speed = oi.speed();
		double speed_ips = map(speed,
								 -1.0,
								  1.0,
								 -DriveConstants.MAX_ALLOWED_SPEED_IPS,
								 DriveConstants.MAX_ALLOWED_SPEED_IPS);
        driveSubsystem.velocityDrive_auto(
            speed_ips,
            turnRate_radps
        );

        return false;
    }
}