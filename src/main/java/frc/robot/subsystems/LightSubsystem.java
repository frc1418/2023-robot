package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LightConstants;
import frc.robot.subsystems.GrabberSubsystem;

public class LightSubsystem {
    Spark blinkin;

    boolean blueAlliance = true;

    private GrabberSubsystem m_GrabberSubsystem;
    
    public LightSubsystem(GrabberSubsystem m_GrabberSubsystem) {
        blinkin = new Spark(LightConstants.BLINKIN_CHANNEL);
        this.m_GrabberSubsystem = m_GrabberSubsystem;
    }
    public void normalLights() {
        if(blueAlliance)
        blinkin.set(LightConstants.BLUE);
        else
        blinkin.set(LightConstants.RED);
    }
    public void balancingLights() {
        blinkin.set(LightConstants.PINK);
    }
    public void balancedLights() {
        blinkin.set(LightConstants.GREEN);
    }

    public void teleopLights() {
        if (m_GrabberSubsystem.getGrabberClosed()) 
        blinkin.set(LightConstants.GOLD);
        else
        blinkin.set(LightConstants.WHITE);
    }

}
