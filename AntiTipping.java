import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.Optional;

/**
 * {@code AntiTipping} provides a proportional correction system to prevent the robot from tipping
 * over during operation.
 *
 * <p>It uses the robot's pitch and roll measurements to detect excessive inclination and computes
 * a correction velocity in the opposite direction of the tilt. The resulting correction can be
 * added to the robot's translational velocity to help stabilize it.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>Instantiate the class with the desired configuration parameters.
 *   <li>Call {@link #calculate(Rotation3d)} periodically (for example, once per control loop).
 *   <li>Use the returned {@link Optional} or {@link #getVelocityAntiTipping()} to apply the
 *       correction to your drive command.
 * </ol>
 *
 * <h2>Configuration</h2>
 * <ul>
 *   <li>{@link #setTippingThreshold(double)} — sets the tipping detection threshold in degrees.
 *   <li>{@link #setMaxCorrectionSpeed(double)} — sets the maximum correction velocity in meters
 *       per second.
 *   <li>{@link #setKp(double)} — sets the proportional gain used to compute the correction.
 * </ul>
 *
 * <p>The correction is purely proportional:
 * {@code correction = kP * inclinationMagnitude}, and it is clamped to
 * {@code maxCorrectionSpeed}.
 *
 * @since 2025
 */
public class AntiTipping {

  private double tippingThresholdDegrees;
  private double maxCorrectionSpeed; // m/s
  private double kP; // proportional gain

  private double pitch = 0.0;
  private double roll = 0.0;
  private double correctionSpeed = 0.0;
  private double inclinationMagnitude = 0.0;
  private double yawDirectionDeg = 0.0;
  private boolean isTipping = false;
  private Rotation2d tiltDirection = new Rotation2d();
  private ChassisSpeeds speeds = new ChassisSpeeds();

  /**
   * Creates a new {@code AntiTipping} instance.
   *
   * @param kP proportional gain for correction
   * @param tippingThresholdDegrees tipping detection threshold in degrees
   * @param maxCorrectionSpeed maximum correction velocity in meters per second
   */
  public AntiTipping(double kP, double tippingThresholdDegrees, double maxCorrectionSpeed) {
    this.kP = kP;
    this.tippingThresholdDegrees = tippingThresholdDegrees;
    this.maxCorrectionSpeed = maxCorrectionSpeed;
  }

  /** Sets the tipping detection threshold in degrees. */
  public void setTippingThreshold(double degrees) {
    this.tippingThresholdDegrees = degrees;
  }

  /** Sets the maximum correction velocity in meters per second. */
  public void setMaxCorrectionSpeed(double speedMetersPerSecond) {
    this.maxCorrectionSpeed = speedMetersPerSecond;
  }

  /** Sets the proportional gain used for correction. */
  public void setKp(double kP) {
    this.kP = kP;
  }

  /**
   * Calculates the anti-tipping correction from the robot rotation.
   *
   * <p>This method updates the internal values for pitch, roll, tilt direction, tilt magnitude,
   * and correction speed. If the robot is beyond the tipping threshold, a correction
   * {@link ChassisSpeeds} is returned; otherwise, the result is empty.
   *
   * @param robotRotation3d the current robot rotation
   * @return an {@link Optional} containing the correction speeds when tipping is detected, or an
   *     empty {@link Optional} otherwise
   */
  public Optional<ChassisSpeeds> calculate(Rotation3d robotRotation3d) {
    pitch = robotRotation3d.getY();
    roll = robotRotation3d.getX();

    double tippingThresholdRadians = Units.degreesToRadians(tippingThresholdDegrees);

    isTipping = Math.hypot(pitch, roll) > tippingThresholdRadians;

    tiltDirection = new Rotation2d(Math.atan2(roll, pitch));
    yawDirectionDeg = tiltDirection.getDegrees();

    inclinationMagnitude = Math.hypot(Units.radiansToDegrees(pitch), Units.radiansToDegrees(roll));

    correctionSpeed = kP * inclinationMagnitude;
    correctionSpeed = MathUtil.clamp(correctionSpeed, -maxCorrectionSpeed, maxCorrectionSpeed);

    Translation2d correctionVector =
        new Translation2d(0, 1).rotateBy(tiltDirection).times(correctionSpeed);

    speeds = new ChassisSpeeds(correctionVector.getX(), -correctionVector.getY(), 0);
    /* --------------------------------------------------- */

    return isTipping ? Optional.of(speeds) : Optional.empty();
  }

  /** Returns the most recent inclination magnitude. */
  public double getLastInclinationMagnitude() {
    return inclinationMagnitude;
  }

  /** Returns the most recent tilt direction in degrees. */
  public double getLastYawDirectionDeg() {
    return yawDirectionDeg;
  }

  /** Returns {@code true} if the robot is currently beyond the tipping threshold. */
  public boolean isTipping() {
    return isTipping;
  }

  /** Returns the latest anti-tipping correction velocity. */
  public ChassisSpeeds getVelocityAntiTipping() {
    return speeds;
  }

  /** Returns the most recent tilt direction as a {@link Rotation2d}. */
  public Rotation2d getLastTiltDirection() {
    return tiltDirection;
  }
}