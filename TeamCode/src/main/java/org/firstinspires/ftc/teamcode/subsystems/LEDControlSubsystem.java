package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * LED Control Subsystem
 *
 * Hardware:
 * - rgb_color_PWM1: PWM LED controller driven via standard servo signal
 * - rgb_color_PWM2: PWM LED controller driven via standard servo signal
 *
 * Provides high-level helpers for configuring both LED channels with shared
 * presets, independent overrides, and simple blinking patterns.
 */
@Configurable
public class LEDControlSubsystem implements Subsystem {

    public static final LEDControlSubsystem INSTANCE = new LEDControlSubsystem();
    private LEDControlSubsystem() {}

    // =============================================
    // CONFIGURABLE CONSTANTS
    // =============================================

    public static double OFF_POSITION = 0.00;
    public static double RED_POSITION = 0.29;
    public static double GREEN_POSITION = 0.50;
    public static double BLUE_POSITION = 0.61;
    public static double WHITE_POSITION = 1.00;
    public static double YELLOW_POSITION = 0.38;
    public static double ORANGE_POSITION = 0.33;
    public static double STROBE_INTERVAL_MS = 250.0;

    // =============================================
    // HARDWARE
    // =============================================

    private Servo led1;
    private Servo led2;

    // =============================================
    // STATE TRACKING
    // =============================================

    private boolean mirrored = true;
    private double led1Target = OFF_POSITION;
    private double led2Target = OFF_POSITION;

    private boolean strobing = false;
    private LedColor strobeColorA = LedColor.OFF;
    private LedColor strobeColorB = LedColor.OFF;
    private double strobeIntervalMs = STROBE_INTERVAL_MS;
    private final ElapsedTime strobeTimer = new ElapsedTime();

    // =============================================
    // INITIALIZATION
    // =============================================

    @Override
    public void initialize() {
        led1 = ActiveOpMode.hardwareMap().get(Servo.class, "rgb_color_PWM1");
        led2 = ActiveOpMode.hardwareMap().get(Servo.class, "rgb_color_PWM2");

        applyTargets();
        strobeTimer.reset();
    }

    // =============================================
    // PERIODIC
    // =============================================

    @Override
    public void periodic() {
        if (strobing && strobeTimer.milliseconds() >= strobeIntervalMs) {
            toggleStrobeColors();
            strobeTimer.reset();
        }
    }

    // =============================================
    // PUBLIC CONTROL METHODS
    // =============================================

    /**
     * Sets both LED controllers to the same servo position.
     */
    public void setBoth(double position) {
        double clipped = Range.clip(position, 0.0, 1.0);
        led1Target = clipped;
        if (mirrored) {
            led2Target = clipped;
        }
        strobing = false;
        applyTargets();
    }

    /**
     * Sets both LED controllers to the same preset color.
     */
    public void setBoth(LedColor color) {
        setBoth(color.position());
    }

    /**
     * Sets individual LED positions.
     */
    public void setIndividual(double led1Position, double led2Position) {
        mirrored = false;
        led1Target = Range.clip(led1Position, 0.0, 1.0);
        led2Target = Range.clip(led2Position, 0.0, 1.0);
        strobing = false;
        applyTargets();
    }

    /**
     * Applies the same preset color to both LEDs but keeps mirrored state.
     */
    public void setColor(LedColor color) {
        setBoth(color);
    }

    public void setLed1(double position) {
        led1Target = Range.clip(position, 0.0, 1.0);
        strobing = false;
        applyTargets();
    }

    public void setLed1(LedColor color) {
        setLed1(color.position());
    }

    public void setLed2(double position) {
        led2Target = Range.clip(position, 0.0, 1.0);
        mirrored = false;
        strobing = false;
        applyTargets();
    }

    public void setLed2(LedColor color) {
        setLed2(color.position());
    }

    /**
     * Enables or disables mirrored mode. When mirrored, LED2 follows LED1
     * whenever {@link #setBoth(double)} is used.
     */
    public void setMirrored(boolean mirrored) {
        this.mirrored = mirrored;
        if (mirrored) {
            led2Target = led1Target;
            applyTargets();
        }
    }

    /**
     * Starts a simple strobe between two colors.
     */
    public void startStrobe(LedColor first, LedColor second, double intervalMs) {
        strobing = true;
        strobeColorA = first;
        strobeColorB = second;
        strobeIntervalMs = Math.max(10.0, intervalMs);
        mirrored = true;
        led1Target = first.position();
        led2Target = first.position();
        applyTargets();
        strobeTimer.reset();
    }

    /**
     * Stops any active strobe or animation and leaves LEDs at their current position.
     */
    public void stopStrobe() {
        strobing = false;
        strobeTimer.reset();
    }

    public boolean isStrobing() {
        return strobing;
    }

    public double getLed1Target() {
        return led1Target;
    }

    public double getLed2Target() {
        return led2Target;
    }

    public boolean isMirrored() {
        return mirrored;
    }

    // =============================================
    // HELPER METHODS
    // =============================================

    private void applyTargets() {
        if (led1 != null) {
            led1.setPosition(led1Target);
        }
        if (led2 != null) {
            led2.setPosition(led2Target);
        }
    }

    private void toggleStrobeColors() {
        if (!strobing) {
            return;
        }

        if (Math.abs(led1Target - strobeColorA.position()) < 1e-6) {
            led1Target = strobeColorB.position();
            led2Target = strobeColorB.position();
        } else {
            led1Target = strobeColorA.position();
            led2Target = strobeColorA.position();
        }

        applyTargets();
    }

    // =============================================
    // COLOR PRESETS
    // =============================================

    public enum LedColor {
        OFF(() -> OFF_POSITION),
        RED(() -> RED_POSITION),
        GREEN(() -> GREEN_POSITION),
        BLUE(() -> BLUE_POSITION),
        WHITE(() -> WHITE_POSITION),
        YELLOW(() -> YELLOW_POSITION),
        ORANGE(() -> ORANGE_POSITION);

        private final DoubleSupplier supplier;

        LedColor(DoubleSupplier supplier) {
            this.supplier = supplier;
        }

        public double position() {
            return Range.clip(supplier.getAsDouble(), 0.0, 1.0);
        }
    }
}

