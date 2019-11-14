package robot;

public class Utilities {
    /**
     * convert meters to radians of the drivetrain wheels
     *
     * @param meters distance in meters.
     * @return wheel radians.
     */
    public static double metersToRadians(double meters) {
        return meters * Constants.Drivetrain.WHEEL_DIAMATER * Math.PI / 2;
    }
}
