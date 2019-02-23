package frc.robot.subsystem.autonomous;



import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.drive.DriveSubsystem;
import frc.robot.utils.CommandUtils;
import frc.robot.utils.Controller;

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

    @Override
    protected boolean isFinished() {
        boolean forceIdle = oi.driverIdle();

        if (forceIdle) {
            return false;
        }



        boolean useAutoAssist = autonomousSubsystem.getUseAutoAssist();

        if (!useAutoAssist) {
            return CommandUtils.stateChange(new Idle());
        }
        


        CameraFeedback feedback = autonomousSubsystem.getClosestObjectData();

        double offAxis = feedback.getOffAxis();
        double parallax = feedback.getParallax();
        double distance = feedback.getDistance();

        // no need to keep guiding if in a certain distance
        if (distance <= AutonomousConstants.GUIDANCE_STOP) {
            return CommandUtils.stateChange(new Idle());
        }

        guidance.setOffAxis(offAxis);
        guidance.setParallax(parallax);



        double turnRate_radps = guidance.getTurnRate(distance);

        driveSubsystem.velocityDrive(
            AutonomousConstants.ASSIST_VELOCITY_IPS,
            turnRate_radps
        );

        return false;
    }
}