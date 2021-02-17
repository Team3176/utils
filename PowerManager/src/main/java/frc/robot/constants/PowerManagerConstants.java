package frc.robot.constants;

public class PowerManagerConstants {
    // Constants for monitoring PDP and power

    //CAN IDs
    public static final int PDP_CAN_ID = 0;

    //PDP CHANNEL IDs
    public static final int DRIVE_ONE_CID = 1;
    public static final int DRIVE_TWO_CID = 2;
    public static final int DRIVE_THREE_CID = 3;
    public static final int DRIVE_FOUR_CID = 4;

    public static final boolean[] CHANNEL_USED = {true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true
    };

    public static final boolean[] UPDATE_CHANNEL = {true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true,
                                                  true
    };

    public static final String[] CHANNEL_NAME = {"PDPChan0",
                                                   "PDPChan1",
                                                   "PDPChan2",
                                                   "PDPChan3",
                                                   "PDPChan4",
                                                   "PDPChan5",
                                                   "PDPChan6",
                                                   "PDPChan7",
                                                   "PDPChan8",
                                                   "PDPChan9",
                                                   "PDPChan10",
                                                   "PDPChan11",
                                                   "PDPChan12",
                                                   "PDPChan13",
                                                   "PDPChan14",
                                                   "PDPChan15"
    };

    public static final double[] CHANNEL_MAX_CURRENT = {40.0,
                                                        40.0, 
                                                        40.0, 
                                                        40.0, 
                                                        40.0, 
                                                        40.0, 
                                                        40.0, 
                                                        40.0, 
                                                        30.0, 
                                                        30.0, 
                                                        30.0, 
                                                        30.0, 
                                                        30.0, 
                                                        30.0, 
                                                        30.0, 
                                                        30.0 
    };


}
