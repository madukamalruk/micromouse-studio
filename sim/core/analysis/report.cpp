#include "sim/core/analysis/report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace mmsim::core::analysis {

std::string html_escape(const std::string& s) {
    std::string o;
    o.reserve(s.size());
    for (const char c : s) {
        switch (c) {
            case '&':
                o += "&amp;";
                break;
            case '<':
                o += "&lt;";
                break;
            case '>':
                o += "&gt;";
                break;
            case '"':
                o += "&quot;";
                break;
            case '\'':
                o += "&#39;";
                break;
            default:
                o += c;
        }
    }
    return o;
}

std::string overall_verdict(const std::vector<ReportCheck>& checks) {
    int fails = 0, measured = 0, total = 0;
    for (const auto& c : checks) {
        if (c.verdict == "INFO") continue;
        ++total;
        fails += c.verdict == "FAIL";
        measured += c.verdict != "NOT MEASURED";
    }
    if (measured == 0) return "NO DATA";
    if (fails) return "FAIL";
    if (measured < total)
        return "PASS (partial: " + std::to_string(measured) + " of " + std::to_string(total) +
               " checks measured)";
    return "PASS";
}

namespace {
std::string num(double v, const char* f = "%.4g") {
    char b[48];
    std::snprintf(b, sizeof b, f, v);
    return b;
}
}  // namespace

std::string svg_plot(const ReportPlot& p, int width, int height) {
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (const auto& s : p.series)
        for (std::size_t i = 0; i < std::min(s.x.size(), s.y.size()); ++i) {
            if (!std::isfinite(s.x[i]) || !std::isfinite(s.y[i])) continue;
            x0 = std::min(x0, s.x[i]);
            x1 = std::max(x1, s.x[i]);
            y0 = std::min(y0, s.y[i]);
            y1 = std::max(y1, s.y[i]);
        }
    std::ostringstream o;
    const int ml = 56, mr = 12, mt = 26, mb = 34;
    o << "<svg viewBox=\"0 0 " << width << ' ' << height
      << "\" class=\"plot\" role=\"img\" aria-label=\"" << html_escape(p.title) << "\">";
    o << "<text x=\"" << ml << "\" y=\"16\" class=\"pt\">" << html_escape(p.title) << "</text>";
    if (x0 > x1) {
        o << "<text x=\"" << ml << "\" y=\"" << height / 2
          << "\" class=\"ax\">no data</text></svg>";
        return o.str();
    }
    if (x1 - x0 < 1e-12) x1 = x0 + 1;
    if (y1 - y0 < 1e-12) {
        y0 -= 1;
        y1 += 1;
    }
    const double pw = width - ml - mr, ph = height - mt - mb;
    auto X = [&](double x) { return ml + (x - x0) / (x1 - x0) * pw; };
    auto Y = [&](double y) { return mt + (1 - (y - y0) / (y1 - y0)) * ph; };
    o << "<rect x=\"" << ml << "\" y=\"" << mt << "\" width=\"" << pw << "\" height=\"" << ph
      << "\" class=\"frame\"/>";
    for (int k = 0; k <= 4; ++k) {
        const double yv = y0 + (y1 - y0) * k / 4.0, xv = x0 + (x1 - x0) * k / 4.0;
        o << "<line x1=\"" << ml << "\" x2=\"" << ml + pw << "\" y1=\"" << num(Y(yv), "%.1f")
          << "\" y2=\"" << num(Y(yv), "%.1f") << "\" class=\"grid\"/>";
        o << "<text x=\"" << ml - 6 << "\" y=\"" << num(Y(yv) + 4, "%.1f")
          << "\" class=\"ax\" text-anchor=\"end\">" << num(yv) << "</text>";
        o << "<text x=\"" << num(X(xv), "%.1f") << "\" y=\"" << height - mb + 16
          << "\" class=\"ax\" text-anchor=\"middle\">" << num(xv) << "</text>";
    }
    o << "<text x=\"" << ml + pw << "\" y=\"" << height - 4
      << "\" class=\"ax\" text-anchor=\"end\">" << html_escape(p.x_label) << "</text>";
    o << "<text x=\"4\" y=\"16\" class=\"ax\">" << html_escape(p.y_label) << "</text>";
    int li = 0;
    for (const auto& s : p.series) {
        const std::size_t n = std::min(s.x.size(), s.y.size());
        const std::size_t step = std::max<std::size_t>(1, n / 1200);
        o << "<polyline fill=\"none\" stroke=\"" << html_escape(s.colour)
          << "\" stroke-width=\"1.5\" points=\"";
        for (std::size_t i = 0; i < n; i += step)
            if (std::isfinite(s.x[i]) && std::isfinite(s.y[i]))
                o << num(X(s.x[i]), "%.1f") << ',' << num(Y(s.y[i]), "%.1f") << ' ';
        o << "\"/>";
        o << "<text x=\"" << width - mr - 4 << "\" y=\"" << mt + 14 + 14 * li++
          << "\" class=\"ax\" text-anchor=\"end\" fill=\"" << html_escape(s.colour) << "\">"
          << html_escape(s.label) << "</text>";
    }
    o << "</svg>";
    return o.str();
}

std::string build_report(const ReportInput& in) {
    const std::string css =
        "body{font:13px/1.45 'Segoe UI',Helvetica,Arial,sans-serif;color:#0f172a;margin:24px "
        "auto;max-width:900px;padding:0 16px}"
        "h1{margin:0;font-size:22px}h2{font-size:15px;border-bottom:1px solid "
        "#cbd5e1;padding-bottom:3px;margin-top:22px}"
        "table{border-collapse:collapse;width:100%;margin:6px 0}td,th{border:1px solid "
        "#e2e8f0;padding:4px 7px;text-align:left;"
        "font-variant-numeric:tabular-nums}th{background:#f1f5f9;font-weight:600}.PASS{color:#"
        "15803d;font-weight:700}"
        ".FAIL{color:#b91c1c;font-weight:700}.NOT{color:#b45309;font-weight:700}.INFO{color:#"
        "475569}"
        ".note{color:#475569;font-size:12px}.hdr{display:flex;justify-content:space-between;align-"
        "items:flex-end;gap:12px}"
        "button{padding:6px 14px;font:inherit}.plot{width:100%;height:auto;margin:8px "
        "0}.pt{font:600 12px sans-serif;fill:#0f172a}"
        ".ax{font:11px "
        "sans-serif;fill:#475569}.grid{stroke:#e2e8f0;stroke-width:1}.frame{fill:none;stroke:#"
        "cbd5e1}"
        "pre{background:#f8fafc;border:1px solid "
        "#e2e8f0;padding:8px;white-space:pre-wrap;font-size:12px}"
        "@media print{button{display:none}body{margin:0}}";
    std::ostringstream o;
    o << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><title>"
      << html_escape(in.title) << "</title><style>" << css << "</style></head><body>";
    o << "<div class=\"hdr\"><div><h1>" << html_escape(in.title)
      << "</h1><div class=\"note\">generated " << html_escape(in.generated)
      << " &middot; simulated run, not a hardware measurement</div></div>"
         "<button onclick=\"window.print()\">Print / save as PDF</button></div>";
    o << "<h2>Run record</h2><table>";
    for (const auto& [k, v] : in.run)
        o << "<tr><th>" << html_escape(k) << "</th><td>" << html_escape(v) << "</td></tr>";
    o << "</table>";
    const std::string overall = overall_verdict(in.checks);
    const char* cls = overall.rfind("PASS", 0) == 0 ? "PASS" : (overall == "FAIL" ? "FAIL" : "NOT");
    o << "<h2>Verdict: <span class=\"" << cls << "\">" << html_escape(overall)
      << "</span></h2><table><tr><th>Check</th><th>Verdict</th><th>Measured</th><th>Limit</th></"
         "tr>";
    for (const auto& c : in.checks) {
        const std::string vc = c.verdict.substr(0, c.verdict.find(' '));
        o << "<tr><td>" << html_escape(c.name) << "</td><td class=\"" << html_escape(vc) << "\">"
          << html_escape(c.verdict) << "</td><td>" << html_escape(c.measured) << "</td><td>"
          << html_escape(c.limit) << "</td></tr>";
    }
    o << "</table><div class=\"note\">Limits are engineering aids chosen for this report, not a "
         "specification.</div>";
    if (!in.metrics.empty()) {
        o << "<h2>Metrics</h2><table>";
        for (const auto& [k, v] : in.metrics)
            o << "<tr><th>" << html_escape(k) << "</th><td>" << html_escape(v) << "</td></tr>";
        o << "</table>";
    }
    for (const auto& p : in.plots) o << svg_plot(p);
    if (!in.events.empty()) {
        o << "<h2>Event log</h2><pre>";
        for (const auto& e : in.events) o << html_escape(e) << '\n';
        o << "</pre>";
    }
    if (!in.notes.empty()) o << "<h2>Notes</h2><p>" << html_escape(in.notes) << "</p>";
    o << "</body></html>";
    return o.str();
}

bool write_matlab_csv(std::ostream& os, const std::vector<double>& t_ms,
                      const std::vector<double>& wl, const std::vector<double>& wr,
                      const std::vector<std::pair<std::string, std::vector<double>>>& extra,
                      int segment) {
    const std::size_t n = t_ms.size();
    if (wl.size() != n || wr.size() != n) return false;
    for (const auto& [name, col] : extra)
        if (col.size() != n) return false;
    os << "t_ms,segment,wheel_l_cps,wheel_r_cps";
    for (const auto& [name, col] : extra) os << ',' << name;
    os << '\n';
    char b[48];
    for (std::size_t i = 0; i < n; ++i) {
        std::snprintf(b, sizeof b, "%.3f", t_ms[i]);
        os << b << ',' << segment;
        std::snprintf(b, sizeof b, ",%.3f,%.3f", wl[i], wr[i]);
        os << b;
        for (const auto& [name, col] : extra) {
            std::snprintf(b, sizeof b, ",%.6g", col[i]);
            os << b;
        }
        os << '\n';
    }
    return static_cast<bool>(os);
}

}  // namespace mmsim::core::analysis
