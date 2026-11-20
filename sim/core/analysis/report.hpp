#pragma once
// Printable offline engineering report and MATLAB tuning CSV (Slice 9.7).

#include <ostream>
#include <string>
#include <vector>

namespace mmsim::core::analysis {

struct ReportCheck {
    std::string name;
    std::string verdict;  // PASS / FAIL / NOT MEASURED / INFO
    std::string measured;
    std::string limit;
};

struct ReportSeries {
    std::string label;
    std::string colour;  // #rrggbb
    std::vector<double> x, y;
};
struct ReportPlot {
    std::string title, x_label, y_label;
    std::vector<ReportSeries> series;
};

struct ReportInput {
    std::string title = "mmsim engineering report";
    std::string generated;                                 // timestamp
    std::vector<std::pair<std::string, std::string>> run;  // run record rows (label, value)
    std::vector<ReportCheck> checks;
    std::vector<std::pair<std::string, std::string>> metrics;
    std::vector<ReportPlot> plots;
    std::vector<std::string> events;  // event log lines
    std::string notes;
};

/// Self-contained HTML (inline CSS + SVG, no external URLs, one print button script).
std::string build_report(const ReportInput& in);
/// Overall verdict: FAIL if any FAIL; PASS (partial ...) if some NOT MEASURED; NO DATA.
std::string overall_verdict(const std::vector<ReportCheck>& checks);

/// HTML escaping of & < > " '.
std::string html_escape(const std::string& s);

/// Inline SVG line plot (width x height px) used by the report.
std::string svg_plot(const ReportPlot& p, int width = 820, int height = 240);

/// CSV for neomouse_pid_fit.m (readtable): t_ms, segment, wheel_l_cps, wheel_r_cps, then extra
/// named columns. All columns must have t_ms's length.
bool write_matlab_csv(std::ostream& os, const std::vector<double>& t_ms,
                      const std::vector<double>& wheel_l_cps,
                      const std::vector<double>& wheel_r_cps,
                      const std::vector<std::pair<std::string, std::vector<double>>>& extra,
                      int segment = 0);

}  // namespace mmsim::core::analysis
