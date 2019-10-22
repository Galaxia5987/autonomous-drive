
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import robot.paths.Test;
import robot.subsystems.Commands.trajectoryTrackerCommand;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private XboxController xbox = new XboxController(2);
    private Button A = new JoystickButton(xbox, 1);
    public Joystick leftStick = new Joystick(0);
    public Joystick rightStick = new Joystick(1);

    public OI() {
        A.whenPressed(new trajectoryTrackerCommand(Test.points, 0, 0, false));
    }


}