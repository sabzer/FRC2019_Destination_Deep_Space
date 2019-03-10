/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.autonomous.motion;

public class MotionPoint {
    public double speed;
    public double turn_radius;
    public MotionPoint(double speed, double turn_radius)
    {
        this.speed=speed;
        this.turn_radius=turn_radius;
    }
}
