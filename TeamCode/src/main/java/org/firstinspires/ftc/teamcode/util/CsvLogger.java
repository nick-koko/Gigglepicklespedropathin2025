package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class CsvLogger {
    private static final String TAG = "CsvLogger";

    private final String baseName;
    private final List<String> rows = new ArrayList<>();

    private boolean started = false;
    private boolean saved = false;
    private long sessionStartMs = 0L;

    public CsvLogger(String baseName) {
        this.baseName = sanitizeFileName(baseName);
    }

    public void start(String headerLine) {
        rows.clear();
        started = true;
        saved = false;
        sessionStartMs = System.currentTimeMillis();

        if (headerLine != null && !headerLine.isEmpty()) {
            rows.add(headerLine);
        }
    }

    public boolean isStarted() {
        return started;
    }

    public long getSessionStartMs() {
        return sessionStartMs;
    }

    public void addRow(String row) {
        if (!started || saved || row == null) return;
        rows.add(row);
    }

    public void addRow(Object... values) {
        if (!started || saved) return;
        rows.add(buildCsvLine(values));
    }

    public int getRowCount() {
        return rows.size();
    }

    public File save() {
        if (!started || saved || rows.isEmpty()) {
            return null;
        }

        File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/turret_logs");
        if (!dir.exists() && !dir.mkdirs()) {
            RobotLog.ee(TAG, "Could not create directory: " + dir.getAbsolutePath());
            return null;
        }

        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date(sessionStartMs));
        File outFile = new File(dir, baseName + "_" + timestamp + ".csv");

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(outFile))) {
            for (String row : rows) {
                writer.write(row);
                writer.newLine();
            }

            writer.flush();
            saved = true;
            RobotLog.ii(TAG, "Saved CSV log: " + outFile.getAbsolutePath());
            return outFile;

        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Failed to save CSV log");
            return null;
        }
    }

    public void discard() {
        rows.clear();
        started = false;
        saved = false;
        sessionStartMs = 0L;
    }

    public static String buildCsvLine(Object... values) {
        StringBuilder sb = new StringBuilder();

        for (int i = 0; i < values.length; i++) {
            if (i > 0) sb.append(',');

            Object value = values[i];
            String s = value == null ? "" : formatValue(value);

            boolean needsQuotes = s.contains(",") || s.contains("\"") || s.contains("\n") || s.contains("\r");
            if (needsQuotes) {
                sb.append('"');
                sb.append(s.replace("\"", "\"\""));
                sb.append('"');
            } else {
                sb.append(s);
            }
        }

        return sb.toString();
    }

    private static String formatValue(Object value) {
        if (value instanceof Double) {
            return String.format(Locale.US, "%.6f", (Double) value);
        }
        if (value instanceof Float) {
            return String.format(Locale.US, "%.6f", (Float) value);
        }
        return String.valueOf(value);
    }

    private static String sanitizeFileName(String input) {
        if (input == null || input.isEmpty()) return "log";
        return input.replaceAll("[^a-zA-Z0-9._-]", "_");
    }
}