package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDColors;

public class LEDSubsystem {
    Spark blinkin;

    public LEDColors ledColor;

    public LEDSubsystem() {
        blinkin = new Spark(LEDConstants.BLINKIN_CHANNEL);
    }


    public void runLEDs() {
        blinkin.set(ledColor.getColor());
    }
    

    public void startColor() {
        if(DriverStation.getAlliance() == Alliance.Blue)
            ledColor = LEDColors.BLUE_ALLIANCE;
        else
            ledColor = LEDColors.RED_ALLIANCE;
    }

    public void clawClosedColor() {
        ledColor = LEDColors.CLAW_CLOSED;
    }

    public void balancingColor() {
        ledColor = LEDColors.BALANCING;
    }

    public void balancedColor() {
        ledColor = LEDColors.BALANCED;
    }

    public void coneColor() {
        ledColor = LEDColors.CONE;
    }

    public void cubeColor() {
        ledColor = LEDColors.CUBE;
    }

}
