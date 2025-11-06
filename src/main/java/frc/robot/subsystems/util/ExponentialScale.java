package frc.robot.subsystems.util;

/**
 * Applies exponential scaling to joystick inputs for finer control at low speeds while maintaining
 * full range at high speeds.
 *
 * <p>The scaling formula is: output = sign(input) * (a * input^2 + (1-a) * input)
 *
 * <p>Where 'a' is the exponential factor (0 to 1): - a = 0: Linear (no scaling) - a = 0.5: Moderate
 * exponential - a = 1.0: Fully quadratic (most aggressive)
 */
public class ExponentialScale {
    private final double m_exponent;

    /**
     * Creates an exponential scaler with the specified exponential factor.
     *
     * @param exponent Exponential factor from 0 to 1 (0 = linear, 1 = fully quadratic). Typical
     *     value: 0.3 to 0.5
     */
    public ExponentialScale(double exponent) {
        m_exponent = Math.max(0.0, Math.min(1.0, exponent)); // Clamp to [0, 1]
    }

    /**
     * Applies exponential scaling to an input value.
     *
     * @param input Raw joystick input (typically -1 to 1)
     * @return Scaled output with same sign but exponential curve
     */
    public double calculate(double input) {
        // Preserve sign
        double sign = Math.signum(input);
        double absInput = Math.abs(input);

        // Apply exponential scaling: mix between linear and quadratic
        // Formula: a * x^2 + (1-a) * x
        double scaled = m_exponent * absInput * absInput + (1.0 - m_exponent) * absInput;

        return sign * scaled;
    }

    /**
     * Gets the current exponential factor.
     *
     * @return The exponential factor (0 to 1)
     */
    public double getExponent() {
        return m_exponent;
    }
}
