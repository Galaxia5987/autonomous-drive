package robot.subsystems;

public class drivetrainConstants {
    public static final double TICKS_PER_METER = 0;
    public static final double TICKS_PER_ROTATION = 0;
    public static final double ROBOT_MASS = 0; //Robot Mass + 5kg for the battery + 2kg for the bumpers

    public static final double WHEEL_RADIUS = 0; // meters. TODO tune
    public static final double ROBOT_WIDTH = 0; // meters

    public static final double kBeta = 0;
    public static final double kZeta = 0;

    public static final double kVDriveLeftLow = 0; // Volts per radians per second - Calculated emperically
    public static final double kADriveLeftLow = 0; // Volts per radians per second per second
    public static final double kVInterceptLeftLow = 0; // Volts

    public static final double kVDriveRightLow = 0; // Volts per radians per second - Calculated emperically
    public static final double kADriveRightLow = 0; // Volts per radians per second per second
    public static final double kVInterceptRightLow = 0; // Volts

    public static final double MOMENT_OF_INERTIA = 10;// kg m^2
    public static final double ANGULAR_DRAG = 12;// N*m / (rad/sec)


}
