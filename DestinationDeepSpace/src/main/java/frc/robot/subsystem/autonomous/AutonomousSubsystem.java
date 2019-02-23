/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.autonomous;

import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.BitBucketSubsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;



public class AutonomousSubsystem extends BitBucketSubsystem {
    private final OI oi = OI.instance();

    // Singleton method; use AutonomousSubsystem.instance() to get the AutonomousSubsystem instance.
	public static AutonomousSubsystem instance() {
		if (inst == null) {
            inst = new AutonomousSubsystem();
        }

		return inst;
	}
    private static AutonomousSubsystem inst;
    




    @Override
	protected void initDefaultCommand() {
    }

    @Override
	public void periodic() {
    }



    @Override
	public void diagnosticsInitialize() {
	}

	@Override
	public void diagnosticsPeriodic() {
    }

    @Override
	public void diagnosticsCheck() {		
    }
    


    private NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
	private NetworkTable bvTable = networkTable.getTable("BucketVision");
	private NetworkTableEntry bvStateEntry = bvTable.getEntry("BucketVisionState");
    private NetworkTableEntry bvCameraNumber = bvTable.getEntry("CameraNum");
    
    public CameraFeedback getClosestObjectData() {
        int numTargets = (int) bvTable.getEntry("NumTargets").getValue().getDouble();
        if (numTargets == 0) {
            return null; // null if no target found (do we want this behavior?)
        }

        double[] distance = bvTable.getEntry("distance").getValue().getDoubleArray();
        double[] pos_x = bvTable.getEntry("pos_x").getValue().getDoubleArray();
        double[] parallax = bvTable.getEntry("parallax").getValue().getDoubleArray();

        int min_index = 0;
        double min_offAxis = 2 * pos_x[0] - 1; // normalize to [-1, 1] from [0, 1]
        for (int i = 0; i < numTargets; i++) {
            double offAxis = 2 * pos_x[min_index] - 1; // normalize to [-1, 1] from [0, 1]

            if (offAxis < min_offAxis) {
                min_index = i;
                min_offAxis = offAxis;
            }
        }

        double offAxis = 2 * pos_x[min_index] - 1; // normalize to [-1, 1] from [0, 1]



        boolean isInAutoAssistRegion = true; // TODO: for now

        return new CameraFeedback(
            isInAutoAssistRegion,
            parallax[min_index],
            offAxis,
            distance[min_index]
        );
    }



    public boolean getUseAutoAssist() {
        return getDiagnosticsEnabled() && SmartDashboard.getBoolean(getName() + "/AutoAssist", false);
    }



    @Override
	public void initialize() {
		initializeBaseDashboard();
    }
    


    public void disable() {

    }
}