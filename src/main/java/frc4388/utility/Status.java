package frc4388.utility;

import java.util.ArrayList;
import java.util.List;

public class Status {
    public enum ReportLevel {
        INFO,
        WARNING,
        ERROR
    }

    public class Report {
        public ReportLevel reportLevel;
        public String description;

        @Override
        public String toString() {
            return this.reportLevel.name() + ": " + this.description;
        }
    }

    public List<Report> reports;

    public Status() {
        this.reports = new ArrayList<>();
    }

    public void addReport(ReportLevel level, String description) {
        Report r = new Report();
        r.reportLevel = level;
        r.description = description;
        this.reports.add(r);
    }

    public boolean hasReport() {
        return reports.size() == 0;
    }
}
