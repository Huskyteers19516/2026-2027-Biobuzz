package org.firstinspires.ftc.teamcode.panels;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

public final class MirroredTelemetry implements Telemetry, TelemetryInternal {

    private static final int MAX_ENTRIES = 255;

    private static final class Entry {
        final String caption;
        final String format;
        final Object[] args;
        final Object value;
        final Func<?> producer;
        Item item;
        Line line;

        Entry(String caption, String format, Object[] args, Object value, Func<?> producer) {
            this.caption = caption;
            this.format = format;
            this.args = args;
            this.value = value;
            this.producer = producer;
        }

        boolean isRetained() {
            if (item != null) {
                return item.isRetained();
            }
            return producer != null;
        }
    }

    private final Telemetry driverStation;
    private final List<Entry> entries = new ArrayList<>();
    private final DecimalFormat decimalFormat = new DecimalFormat("0.####");
    private boolean clearOnAdd;
    private volatile Entry[] unsent;

    public MirroredTelemetry(Telemetry driverStation) {
        this.driverStation = driverStation;
    }

    public Telemetry driverStation() {
        return driverStation;
    }

    private void record(Entry entry) {
        synchronized (entries) {
            if (clearOnAdd) {
                removeNonRetained();
                clearOnAdd = false;
            }
            if (entries.size() < MAX_ENTRIES) {
                entries.add(entry);
            }
        }
    }

    private void removeNonRetained() {
        entries.removeIf(entry -> !entry.isRetained());
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        Entry entry = new Entry(caption, format, args, null, null);
        entry.item = driverStation.addData(caption, format, args);
        record(entry);
        return entry.item;
    }

    @Override
    public Item addData(String caption, Object value) {
        Entry entry = new Entry(caption, null, null, value, null);
        entry.item = driverStation.addData(caption, value);
        record(entry);
        return entry.item;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        Entry entry = new Entry(caption, null, null, null, valueProducer);
        entry.item = driverStation.addData(caption, valueProducer);
        record(entry);
        return entry.item;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        Entry entry = new Entry(caption, format, null, null, valueProducer);
        entry.item = driverStation.addData(caption, format, valueProducer);
        record(entry);
        return entry.item;
    }

    @Override
    public Line addLine() {
        Entry entry = new Entry(null, null, null, "", null);
        entry.line = driverStation.addLine();
        record(entry);
        return entry.line;
    }

    @Override
    public Line addLine(String lineCaption) {
        Entry entry = new Entry(null, null, null, String.valueOf(lineCaption), null);
        entry.line = driverStation.addLine(lineCaption);
        record(entry);
        return entry.line;
    }

    @Override
    public boolean removeItem(Item item) {
        synchronized (entries) {
            entries.removeIf(entry -> entry.item != null && entry.item == item);
        }
        return driverStation.removeItem(item);
    }

    @Override
    public boolean removeLine(Line line) {
        synchronized (entries) {
            entries.removeIf(entry -> entry.line != null && entry.line == line);
        }
        return driverStation.removeLine(line);
    }

    @Override
    public void clear() {
        driverStation.clear();
        synchronized (entries) {
            clearOnAdd = false;
            removeNonRetained();
        }
    }

    @Override
    public void clearAll() {
        driverStation.clearAll();
        synchronized (entries) {
            clearOnAdd = false;
            entries.clear();
        }
    }

    @Override
    public boolean update() {
        boolean sent = driverStation.update();
        publishToPanels();
        synchronized (entries) {
            clearOnAdd = driverStation.isAutoClear();
        }
        return sent;
    }

    private void publishToPanels() {
        try {
            if (!PanelsBridge.hasViewers()) {
                unsent = null;
                return;
            }
            TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
            if (!panels.getLines().isEmpty()) {
                unsent = null;
                return;
            }
            Entry[] snapshot;
            synchronized (entries) {
                snapshot = entries.toArray(new Entry[0]);
            }
            if (!panels.getShouldUpdateLines()) {
                unsent = snapshot;
                return;
            }
            unsent = null;
            send(panels, snapshot);
        } catch (OpModeManagerImpl.ForceStopException stop) {
            throw stop;
        } catch (Exception | LinkageError ignored) {
        }
    }

    void flushUnsent() {
        Entry[] snapshot = unsent;
        unsent = null;
        if (snapshot == null) {
            return;
        }
        try {
            if (!PanelsBridge.hasViewers()) {
                return;
            }
            TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
            if (!panels.getLines().isEmpty()) {
                return;
            }
            panels.setLastUpdate(0L);
            send(panels, snapshot);
        } catch (Exception | LinkageError ignored) {
        }
    }

    private void send(TelemetryManager panels, Entry[] snapshot) {
        if (snapshot.length == 0) {
            return;
        }
        String separator = driverStation.getCaptionValueSeparator();
        for (Entry entry : snapshot) {
            panels.addLine(render(entry, separator));
        }
        panels.update();
    }

    private String render(Entry entry, String separator) {
        String value;
        try {
            if (entry.producer != null) {
                Object produced = entry.producer.value();
                value = entry.format == null ? String.valueOf(produced) : String.format(entry.format, produced);
            } else if (entry.format != null) {
                value = String.format(entry.format, entry.args);
            } else {
                value = formatValue(entry.value);
            }
        } catch (OpModeManagerImpl.ForceStopException stop) {
            throw stop;
        } catch (Exception error) {
            value = entry.format == null ? "" : entry.format;
        }
        if (entry.caption == null) {
            return value;
        }
        return entry.caption + separator + value;
    }

    private String formatValue(Object value) {
        if (value == null) {
            return "";
        }
        if (value instanceof Double || value instanceof Float) {
            return decimalFormat.format(value);
        }
        return value.toString();
    }

    @Override
    public Object addAction(Runnable action) {
        return driverStation.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return driverStation.removeAction(token);
    }

    @Override
    public void speak(String text) {
        driverStation.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        driverStation.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean isAutoClear() {
        return driverStation.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        driverStation.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return driverStation.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        driverStation.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return driverStation.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        driverStation.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return driverStation.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        driverStation.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        driverStation.setDisplayFormat(displayFormat);
    }

    @Override
    public void setNumDecimalPlaces(int minDecimalPlaces, int maxDecimalPlaces) {
        driverStation.setNumDecimalPlaces(minDecimalPlaces, maxDecimalPlaces);
    }

    @Override
    public Log log() {
        return driverStation.log();
    }

    @Override
    public boolean tryUpdateIfDirty() {
        if (driverStation instanceof TelemetryInternal) {
            return ((TelemetryInternal) driverStation).tryUpdateIfDirty();
        }
        return false;
    }

    @Override
    public void resetTelemetryForOpMode() {
        synchronized (entries) {
            clearOnAdd = false;
            entries.clear();
        }
        if (driverStation instanceof TelemetryInternal) {
            ((TelemetryInternal) driverStation).resetTelemetryForOpMode();
        }
    }
}
