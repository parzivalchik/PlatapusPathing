package com.platapus.pathing.tuning;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;

/**
 * Persists gains to a properties file.
 *
 * <p>Plain {@code java.io} and {@code java.util.Properties}, so it lives in
 * {@code :core} and is testable against a temporary directory with no device.
 * On a robot, point it somewhere on the Control Hub's storage -- the {@code
 * :ftc} module has a helper that picks the conventional FTC location.
 *
 * <p>A properties file rather than JSON on purpose: no dependency, and the
 * result is three human-readable lines a team can read, edit, or delete when
 * they want to start the fit over.
 */
public final class FileTuningStore implements TuningStore {

    private static final String KEY_KS = "kS";
    private static final String KEY_KV = "kV";
    private static final String KEY_KA = "kA";

    private final File file;

    public FileTuningStore(File file) {
        if (file == null) {
            throw new IllegalArgumentException("file must be non-null");
        }
        this.file = file;
    }

    public FileTuningStore(String path) {
        this(new File(path));
    }

    @Override
    public FeedforwardGains load() {
        if (!file.isFile()) {
            return null;
        }

        Properties properties = new Properties();
        try (InputStream in = new FileInputStream(file)) {
            properties.load(in);
        } catch (IOException e) {
            return null;
        }

        try {
            FeedforwardGains gains = new FeedforwardGains(
                    Double.parseDouble(properties.getProperty(KEY_KS, "NaN")),
                    Double.parseDouble(properties.getProperty(KEY_KV, "NaN")),
                    Double.parseDouble(properties.getProperty(KEY_KA, "NaN")));

            // A corrupt or hand-mangled file must not put nonsense into the
            // feedforward. Refusing to load is recoverable; loading a kV of NaN
            // and driving on it is not.
            return gains.isPlausible() ? gains : null;
        } catch (NumberFormatException | NullPointerException e) {
            return null;
        }
    }

    @Override
    public boolean save(FeedforwardGains gains) {
        if (gains == null || !gains.isPlausible()) {
            return false;
        }

        Properties properties = new Properties();
        properties.setProperty(KEY_KS, Double.toString(gains.kS));
        properties.setProperty(KEY_KV, Double.toString(gains.kV));
        properties.setProperty(KEY_KA, Double.toString(gains.kA));

        File parent = file.getParentFile();
        if (parent != null && !parent.isDirectory() && !parent.mkdirs()) {
            return false;
        }

        try (OutputStream out = new FileOutputStream(file)) {
            // The header is a comment line in the file; Properties never reads
            // it back, so it is free to carry the library's old name along.
            properties.store(out, "PlatapusPathing fitted feedforward constants"
                    + " -- in memory of YakuLohi");
            return true;
        } catch (IOException e) {
            // A Control Hub with a full or read-only filesystem is a nuisance,
            // not a reason to abort an auto.
            return false;
        }
    }

    /** Deletes the saved file, so the next fit starts fresh. */
    public boolean clear() {
        return !file.exists() || file.delete();
    }

    public File getFile() {
        return file;
    }
}
