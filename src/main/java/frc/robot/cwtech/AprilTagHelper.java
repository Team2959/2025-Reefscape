package frc.robot.cwtech;

import frc.robot.subsystems.LimelightHelpers;

public class AprilTagHelper {
    public static int tidFromLimelight()
    {
        return (int)LimelightHelpers.getFiducialID("limelight");
    }

    public static int reefAngleFromTid(int tid)
    {
        if (tid == 9 || tid == 22)
            return 300;
        else if (tid == 8 || tid == 17)
            return 240;
        else if (tid == 7 || tid == 18)
            return  180;
        else if (tid == 6 || tid == 19)
            return 120;
        else if (tid == 11 || tid == 20)
            return 60;
        else if (tid == 21 || tid == 10)
            return 0;
        else
            return -1;
    }

    public static int intakeAngleFromTid (int tid)
    {

        if (tid == 8 || tid == 17)
            return 246;
        else if (tid == 6 || tid == 19)
            return 126;
        else
            return -1;
    }

    public static double intakeTargetZFromTid (int tid)
    {
        if (tid == 8) 
            return 3.5;
        else if (tid == 17)
            return 3.5;
        else if (tid == 6) 
            return 3.51;
        else if (tid == 19)
            return 3.53;
        else
        {
            // we should never get here, it should already be one of the other tids
            return 3.2;
        }
    }

    public static double intakeTargetXFromTid (int tid)
    {
        if (tid == 8) 
            return 0.59;
        else if (tid == 17)
            return 0.52;
        else if (tid == 6) 
            return -0.46;
        else if (tid == 19)
            return -0.35;
        else
        {
            // we should never get here, it should already be one of the other tids
            return 0;
        }
    }
}
