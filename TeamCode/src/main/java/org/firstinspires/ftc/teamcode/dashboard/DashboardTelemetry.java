package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DashboardTelemetry implements Telemetry {
    private static final DashboardTelemetry instance = new DashboardTelemetry();

    public static DashboardTelemetry getInstance() {
        return instance;
    }

    private TelemetryPacket currentPacket;
    private LogAdapter log;

    public DashboardTelemetry() {
        currentPacket = new TelemetryPacket(false);
        log = new LogAdapter(currentPacket);
    }

    public TelemetryPacket getCurrentPacket() {
        return currentPacket;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        currentPacket.put(caption, value);
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        throw new UnsupportedOperationException();
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeItem(Item item) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void clear() {
        FtcDashboard.getInstance().clearTelemetry();

        currentPacket = new TelemetryPacket(false);
        log = new LogAdapter(currentPacket);
    }

    @Override
    public void clearAll() {
        clear();
    }

    @Override
    public Object addAction(Runnable action) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeAction(Object token) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void speak(String text) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean update() {
        FtcDashboard.getInstance().sendTelemetryPacket(currentPacket);

        currentPacket = new TelemetryPacket();
        log = new LogAdapter(currentPacket);

        return true;
    }

    @Override
    public Line addLine() {
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        currentPacket.addLine(lineCaption);
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getMsTransmissionInterval() {
        return FtcDashboard.getInstance().getTelemetryTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        FtcDashboard.getInstance().setTelemetryTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return null;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {

    }

    @Override
    public String getCaptionValueSeparator() {
        return null;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {

    }

    @Override
    public Log log() {
        return log;
    }

    private static class LogAdapter implements Telemetry.Log {
        private final TelemetryPacket telemetryPacket;

        private LogAdapter(TelemetryPacket packet) {
            telemetryPacket = packet;
        }

        @Override
        public int getCapacity() {
            return 0;
        }

        @Override
        public void setCapacity(int capacity) {

        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return DisplayOrder.OLDEST_FIRST;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {

        }

        @Override
        public void add(String entry) {
            telemetryPacket.addLine(entry);
        }

        @Override
        public void add(String format, Object... args) {
            telemetryPacket.addLine(String.format(format, args));
        }

        @Override
        public void clear() {
            telemetryPacket.clearLines();
        }
    }
}
