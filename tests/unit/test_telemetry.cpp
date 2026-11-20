// Slice 6: bounded telemetry recorder, CSV export, SHA-256.
#include <doctest/doctest.h>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "sim/core/sha256.hpp"
#include "sim/host/telemetry.hpp"

using namespace mmsim;
using host::Ring;
using host::TelemetryRecorder;

namespace {
std::vector<std::string> lines(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream in(s);
    for (std::string l; std::getline(in, l);) out.push_back(l);
    return out;
}
}  // namespace

TEST_SUITE("telemetry") {
    TEST_CASE("ring: keeps the newest N, oldest first, memory fixed") {
        Ring<int> r(4);
        CHECK(r.empty());
        for (int i = 1; i <= 3; ++i) r.push(i);
        CHECK(r.size() == 3);
        CHECK(r[0] == 1);
        CHECK(r.back() == 3);
        for (int i = 4; i <= 10; ++i) r.push(i);
        CHECK(r.size() == 4);
        CHECK(r.capacity() == 4);
        CHECK(r[0] == 7);
        CHECK(r[3] == 10);
        r.clear();
        CHECK(r.size() == 0);
        Ring<int> none(0);
        none.push(1);  // zero capacity: ignored, no crash
        CHECK(none.size() == 0);
    }

    TEST_CASE("recorder: bounded per channel and in channel count") {
        TelemetryRecorder rec(1000);
        for (int i = 0; i < 5000; ++i) rec.add("sim.v", i * 0.001, static_cast<float>(i));
        const auto* c = rec.channel("sim.v");
        REQUIRE(c);
        CHECK(c->size() == 1000);
        CHECK(c->v[0] == 4000.0f);
        CHECK(c->t[0] == doctest::Approx(4.0));
        for (std::size_t i = 0; i < TelemetryRecorder::kMaxChannels + 10; ++i)
            rec.add("ch" + std::to_string(i), 0.0, 1.0f);
        CHECK(rec.channels().size() == TelemetryRecorder::kMaxChannels);
        CHECK(rec.dropped_channels() > 0);
        CHECK(rec.add("one too many", 0.0, 1.0f) == nullptr);
        CHECK(rec.channel("sim.v") != nullptr);  // existing channels keep working
    }

    TEST_CASE("recorder: lower_bound and sample-and-hold lookup") {
        TelemetryRecorder rec(100);
        for (int i = 0; i < 10; ++i) rec.add("a", i * 0.1, static_cast<float>(i));
        const auto* c = rec.channel("a");
        CHECK(c->lower_bound(-1.0) == 0);
        CHECK(c->lower_bound(0.25) == 3);
        CHECK(c->lower_bound(5.0) == 10);
        CHECK(std::isnan(c->value_at(-0.1)));
        CHECK(c->value_at(0.25) == 2.0f);
        CHECK(c->value_at(0.35) == 3.0f);
        CHECK(c->value_at(9.0) == 9.0f);
    }

    TEST_CASE("csv: a 10 s window at 1 kHz has 10 000 rows, header, held firmware channel") {
        TelemetryRecorder rec;
        for (int i = 1; i <= 20000; ++i) {
            const double t = i / 1000.0;  // exact at whole ms
            rec.add("sim.v", t, static_cast<float>(i));
            if (i % 10 == 0) rec.add("v_set", t, static_cast<float>(i) * 2.0f);  // 100 Hz
            rec.add("sim.d.left", t, i % 2 ? NAN : 50.0f);
        }
        std::ostringstream out;
        const std::size_t rows = host::write_csv(rec, out, 5.0, 15.0);
        CHECK(rows == 10000);
        const auto ls = lines(out.str());
        REQUIRE(ls.size() == 10001);
        CHECK(ls[0] == "t_s,sim.v,sim.d.left,v_set");  // channel creation order
        // First row is t = 5.000 (i = 5000): v_set held from i = 5000, distance present (even).
        CHECK(ls[1] == "5.000,5000,50,10000");
        // Odd i -> NaN distance -> empty cell; v_set held from the last 100 Hz sample.
        CHECK(ls[2] == "5.001,5001,,10000");
        // Channel selection keeps the requested order.
        std::ostringstream sel;
        CHECK(host::write_csv(rec, sel, 0.0, 0.0105, {"v_set", "sim.v"}) == 10);
        const auto ss = lines(sel.str());
        CHECK(ss[0] == "t_s,v_set,sim.v");
        CHECK(ss[1] == "0.001,,1");  // before the first v_set sample: empty
    }

    TEST_CASE("sha256: FIPS 180-4 test vectors, incremental == one-shot") {
        CHECK(core::sha256_hex("") ==
              "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        CHECK(core::sha256_hex("abc") ==
              "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        CHECK(core::sha256_hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
              "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
        core::Sha256 h;
        const std::string big(1000, 'a');
        for (int i = 0; i < 1000; ++i) h.update(big);  // one million 'a'
        CHECK(h.hex() == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
        CHECK(h.hex() == h.hex());  // digest() does not consume the state
    }
}
