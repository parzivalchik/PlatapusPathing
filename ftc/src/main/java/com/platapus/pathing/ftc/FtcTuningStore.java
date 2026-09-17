package com.platapus.pathing.ftc;

import com.platapus.pathing.tuning.FeedforwardGains;
import com.platapus.pathing.tuning.FileTuningStore;
import com.platapus.pathing.tuning.TuningStore;

import java.io.File;

/**
 * A {@link TuningStore} pointed at the conventional FTC settings directory on
 * the Control Hub, so fitted constants survive between OpMode runs and across
 * app restarts.
 *
 * <p>Files under {@code /sdcard/FIRST/} are where the SDK itself keeps
 * configuration, they survive an app update, and a team can pull or delete them
 * over ADB without rebuilding -- which matters when the answer to "why is our
 * auto suddenly different" is "it learned something at the last competition".
 */
public final class FtcTuningStore implements TuningStore {

    private static final String DEFAULT_DIRECTORY = "/sdcard/FIRST/platapuspathing";
    private static final String DEFAULT_FILENAME = "feedforward.properties";

    private final FileTuningStore delegate;

    /** The default location, {@code /sdcard/FIRST/platapuspathing/feedforward.properties}. */
    public FtcTuningStore() {
        this(new File(DEFAULT_DIRECTORY, DEFAULT_FILENAME));
    }

    /**
     * A named file in the default directory. Use this to keep separate fits for
     * separate robots or configurations.
     */
    public FtcTuningStore(String name) {
        this(new File(DEFAULT_DIRECTORY, name.endsWith(".properties")
                ? name : name + ".properties"));
    }

    public FtcTuningStore(File file) {
        this.delegate = new FileTuningStore(file);
    }

    @Override
    public FeedforwardGains load() {
        return delegate.load();
    }

    @Override
    public boolean save(FeedforwardGains gains) {
        return delegate.save(gains);
    }

    /** Deletes the saved fit, so tuning starts from the compiled-in guess again. */
    public boolean clear() {
        return delegate.clear();
    }

    /** Where this store reads and writes. Worth putting on telemetry. */
    public File getFile() {
        return delegate.getFile();
    }
}
