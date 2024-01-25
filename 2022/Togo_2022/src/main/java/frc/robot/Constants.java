// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {

  public static final int NEO_LEFT_FRONT_CANID = 11;
  public static final int NEO_LEFT_BACK_CANID = 41;
  public static final int NEO_RIGHT_FRONT_CANID = 31;
  public static final int NEO_RIGHT_BACK_CANID = 21;

  public static final int XBOX_CONTROLLER_PORT = 2;
  public static final int JOYSTICK_LEFT_PORT = 0;
  public static final int JOYSTICK_RIGHT_PORT = 1;

  public static final String kDriveModeNameSB = "Drive Mode";
  public static final String kControllerTypeNameSB = "Controller Type";

  public static String d_a = "driveMode_arcade";
  public static String d_t = "driveMode_tank";

  // PRESET VALUES -- [0] = index for that set of values (used for referencing the set),
  // [1] = 0 or 1 (0 = joysticks and 1 = xbox controller), [2] = deadband (decimal percent),
  // [3] = scaling constant, [4] = max percent (decimal) without turbo on, [5] = percent (decimal) per second ramp
  public static final double[][] controllerChoices = { {0, 0,   0.10, 1.0, 0.70, 6.0}, // joystick general
                                                        {1, 1,  0.05, 1.0, 0.70, 6.0}, // Xbox general
                                                        {2, 0,  0.10, 1.5, 0.70, 6.0}, // joystick scaled up (L2 & R4)
                                                        {3, 1,  0.10, 1.5, 0.70, 6.0}, // Xbox X3
                                                        {4, 0,  0.10, 2.0, 0.75, 6.0}, // joystick hyper (L2 & R4)
                                                        {5, 1,  0.10, 2.0, 0.75, 6.0} }; // Xbox X3 Hyper
  // To add a new preset, make the list in the format above and name it, making the value at index 0 one more than the value of
  // the most recent preset list created. Then, go to Robot.java and add presetListName[0] as a new option to controlTypeChooser
}

/*
Scaling constant testing on the cart: {0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0}
  - Xbox controller X2 was best at 2.0 and 3.0 and deadband 0.05
  - Xbox controller X1 was best at 0.75 and 1.0 and deadband 0.05
  - Xbox controller X3 was best at 0.75 and deadband 0.05
  - Joysticks J-R2 and J-R3 set were best at 1.0, and 1.5 and deadband 0.20
  - Joysticks J-N1 and J-N2 were best at 1.0 and deadband 0.10
  - Joysticks that were Chris's (not labeled, but L2 and R4?) were best at 1.5 and deadband 0.05
*/
