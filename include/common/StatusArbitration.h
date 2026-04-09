#ifndef STATUS_ARBITRATION_H
#define STATUS_ARBITRATION_H

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <limits>
#include <string>
#include <string_view>

namespace StatusArbitration {

// Status update sources used to arbitrate multi-source robot status feeds.
enum class StatusSource {
    UNKNOWN = 0,
    // Continuous status stream, e.g. SendRobotStatusInfos.
    STATUS_STREAM = 1,
    // Snapshot attached to other requests, e.g. agvStatusList inside AssignmentTaskRequest.
    TASK_SNAPSHOT = 2,
};

struct Policy {
    // Drop updates whose timestamp goes backwards (when both old/new timestamps are valid).
    bool dropRegressingTimestamp = true;
    // When timestamps are equal/coarse (or missing), do not let TASK_SNAPSHOT override STATUS_STREAM.
    bool preferStatusStreamOnEqualTimestamp = true;
};

inline bool ShouldAcceptStatusUpdate(StatusSource oldSource,
                                    int oldTimestampSec,
                                    StatusSource newSource,
                                    int newTimestampSec,
                                    Policy policy = {}) {
    if (policy.dropRegressingTimestamp) {
        if (oldTimestampSec > 0 && newTimestampSec > 0 && newTimestampSec < oldTimestampSec) {
            return false;
        }
    }
    if (policy.preferStatusStreamOnEqualTimestamp) {
        if (oldSource == StatusSource::STATUS_STREAM && newSource == StatusSource::TASK_SNAPSHOT) {
            if (newTimestampSec <= oldTimestampSec) {
                return false;
            }
        }
    }
    return true;
}

inline int NormalizeEpochSeconds(std::int64_t raw) {
    // Heuristic: >= 1e11 is very likely milliseconds-since-epoch.
    if (raw > 100000000000LL) {
        raw /= 1000LL;
    }
    if (raw <= 0) return 0;
    const std::int64_t maxInt = static_cast<std::int64_t>(std::numeric_limits<int>::max());
    if (raw > maxInt) return std::numeric_limits<int>::max();
    return static_cast<int>(raw);
}

inline std::time_t timegm_utc(std::tm* tm) {
#if defined(_WIN32)
    return _mkgmtime(tm);
#else
    return timegm(tm);
#endif
}

inline bool ParseIso8601UtcToEpochSeconds(std::string_view raw, std::int64_t& outSec) {
    std::string s(raw);
    int year = 0, mon = 0, day = 0, hour = 0, minute = 0, second = 0;
    int ms = 0;
    char tz = '\0';

    auto try_parse = [&](const char* fmt, bool hasTz, bool& ok) {
        if (hasTz) {
            ok = (std::sscanf(s.c_str(), fmt, &year, &mon, &day, &hour, &minute, &second, &ms, &tz) >= 7);
        } else {
            ok = (std::sscanf(s.c_str(), fmt, &year, &mon, &day, &hour, &minute, &second) == 6);
        }
    };

    bool ok = false;
    // With milliseconds + 'Z'
    try_parse("%d-%d-%dT%d:%d:%d.%d%c", true, ok);
    if (!ok) {
        try_parse("%d-%d-%d %d:%d:%d.%d%c", true, ok);
    }
    if (ok && tz != 'Z') ok = false;

    // Without milliseconds, with optional 'Z'
    if (!ok) {
        tz = 'Z';
        int parsed = std::sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d%c", &year, &mon, &day, &hour, &minute, &second, &tz);
        if (parsed == 6) {
            ok = true;
        } else if (parsed == 7 && tz == 'Z') {
            ok = true;
        }
    }
    if (!ok) {
        tz = 'Z';
        int parsed = std::sscanf(s.c_str(), "%d-%d-%d %d:%d:%d%c", &year, &mon, &day, &hour, &minute, &second, &tz);
        if (parsed == 6) {
            ok = true;
        } else if (parsed == 7 && tz == 'Z') {
            ok = true;
        }
    }

    if (!ok) return false;
    if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) {
        return false;
    }

    std::tm tm{};
    tm.tm_year = year - 1900;
    tm.tm_mon = mon - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    tm.tm_isdst = 0;
    std::time_t t = timegm_utc(&tm);
    if (t == static_cast<std::time_t>(-1)) return false;
    outSec = static_cast<std::int64_t>(t);
    return true;
}

inline int ParseUpdateTimeSec(std::string_view raw) {
    auto l = raw.find_first_not_of(" \t\r\n");
    if (l == std::string_view::npos) return 0;
    auto r = raw.find_last_not_of(" \t\r\n");
    raw = raw.substr(l, r - l + 1);
    if (raw.empty()) return 0;

    bool digitOnly = std::all_of(raw.begin(), raw.end(), [](unsigned char c) { return std::isdigit(c); });
    if (digitOnly) {
        try {
            std::int64_t v = std::stoll(std::string(raw));
            return NormalizeEpochSeconds(v);
        } catch (...) {
            return 0;
        }
    }

    std::int64_t sec = 0;
    if (ParseIso8601UtcToEpochSeconds(raw, sec)) {
        return NormalizeEpochSeconds(sec);
    }
    return 0;
}

}  // namespace StatusArbitration

#endif  // STATUS_ARBITRATION_H

