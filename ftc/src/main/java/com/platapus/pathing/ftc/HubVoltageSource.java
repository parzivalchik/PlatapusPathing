package com.platapus.pathing.ftc;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.platapus.pathing.tuning.VoltageSource;

/**
 * Battery voltage from the Control Hub.
 *
 * <p>Reads are cached briefly. The voltage sensor goes over the hub's I2C-ish
 * bus and costs real loop time, while the battery does not meaningfully change
 * inside 100 ms -- and both the tuner and the stall detector want it every
 * single loop.
 */
public final class HubVoltageSource implements VoltageSource {

    private static final double CACHE_SECONDS = 0.1;

    private final VoltageSensor sensor;
    private double cachedVoltage;
    private long lastReadNanos;

    public HubVoltageSource(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("hardwareMap must be non-null");
        }
        // Any hub will do; they all report the same battery.
        this.sensor = hardwareMap.voltageSensor.iterator().next();
        this.cachedVoltage = sensor.getVoltage();
        this.lastReadNanos = System.nanoTime();
    }

    public HubVoltageSource(VoltageSensor sensor) {
        if (sensor == null) {
            throw new IllegalArgumentException("sensor must be non-null");
        }
        this.sensor = sensor;
        this.cachedVoltage = sensor.getVoltage();
        this.lastReadNanos = System.nanoTime();
    }

    @Override
    public double getVoltage() {
        long now = System.nanoTime();
        if ((now - lastReadNanos) * 1e-9 >= CACHE_SECONDS) {
            double reading = sensor.getVoltage();
            // The sensor occasionally returns nonsense mid-read; keeping the last
            // good value beats handing a zero to the stall detector.
            if (reading > 6.0 && reading < 20.0) {
                cachedVoltage = reading;
            }
            lastReadNanos = now;
        }
        return cachedVoltage;
    }
}
