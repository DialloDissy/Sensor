#include "threadWorker.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr uint8_t valid_flag = 1;
constexpr uint64_t ms = 1'000'000ULL;
constexpr uint64_t camera_tolerance = 50 * ms;
constexpr uint64_t imu_tolerance = 20 * ms;
constexpr uint64_t gps_tolerance = 1'000 * ms;
constexpr uint64_t imu_window = 100 * ms;
constexpr uint64_t reorder_tolerance = 2 * ms;

struct Synchronizer {
    std::deque<Record> cameras;
    std::deque<Record> imus;
    std::deque<Record> gps;
    std::deque<Record> buttons;
    uint64_t newest_timestamp = 0;
    bool has_timestamp = false;

    static uint64_t distance(uint64_t left, uint64_t right) {
        return left >= right ? left - right : right - left;
    }

    static bool before(const Record& left, const Record& right) {
        return left.timestamp < right.timestamp ||
               (left.timestamp == right.timestamp && left.sequence < right.sequence);
    }

    void add(const Record& record) {
        if (!has_timestamp || record.timestamp > newest_timestamp) {
            newest_timestamp = record.timestamp;
            has_timestamp = true;
        }

        switch (record.sensor_id) {
        case 1:
            cameras.push_back(record);
            break;
        case 2:
            if (record.x >= 0 && record.x <= 35999) {
                imus.push_back(record);
            }
            break;
        case 3:
            gps.push_back(record);
            break;
        case 5:
            if (record.x == 1) {
                buttons.push_back(record);
            }
            break;
        default:
            break;
        }
    }

    bool ready(const Record& button) const {
        return newest_timestamp >= button.timestamp + imu_window + reorder_tolerance;
    }

    static const Record* nearest(const std::deque<Record>& samples,
                                 uint64_t timestamp, uint64_t tolerance) {
        const Record* result = nullptr;
        for (const Record& sample : samples) {
            if (distance(sample.timestamp, timestamp) > tolerance) {
                continue;
            }
            if (result == nullptr || distance(sample.timestamp, timestamp) <
                                         distance(result->timestamp, timestamp) ||
                (distance(sample.timestamp, timestamp) ==
                     distance(result->timestamp, timestamp) &&
                 before(sample, *result))) {
                result = &sample;
            }
        }
        return result;
    }

    static const Record* latest_gps(const std::deque<Record>& samples,
                                    uint64_t timestamp) {
        const Record* result = nullptr;
        for (const Record& sample : samples) {
            if (sample.timestamp > timestamp ||
                timestamp - sample.timestamp > gps_tolerance) {
                continue;
            }
            if (result == nullptr || before(*result, sample)) {
                result = &sample;
            }
        }
        return result;
    }

    static void print_camera(const Record* record) {
        if (record == nullptr) {
            std::cout << "null";
            return;
        }
        std::cout << "{\"timestamp_ns\":" << record->timestamp
                  << ",\"frame\":" << record->x << "}";
    }

    static void print_imu(const Record* record) {
        if (record == nullptr) {
            std::cout << "null";
            return;
        }
        std::cout << "{\"timestamp_ns\":" << record->timestamp
                  << ",\"yaw_cd\":" << record->x
                  << ",\"pitch_cd\":" << record->y
                  << ",\"roll_cd\":" << record->z << "}";
    }

    static void print_gps(const Record* record) {
        if (record == nullptr) {
            std::cout << "null";
            return;
        }
        std::cout << "{\"timestamp_ns\":" << record->timestamp
                  << ",\"lat_e7\":" << record->x
                  << ",\"lon_e7\":" << record->y
                  << ",\"alt_mm\":" << record->z << "}";
    }

    void emit(const Record& button) {
        const Record* camera = nearest(cameras, button.timestamp, camera_tolerance);
        const Record* imu = nearest(imus, button.timestamp, imu_tolerance);
        const Record* gps_sample = latest_gps(gps, button.timestamp);

        std::vector<Record> window;
        for (const Record& sample : imus) {
            if (distance(sample.timestamp, button.timestamp) <= imu_window) {
                window.push_back(sample);
            }
        }
        std::sort(window.begin(), window.end(), before);

        std::cout << "{\"event_id\":" << button.sequence
                  << ",\"timestamp_ns\":" << button.timestamp
                  << ",\"camera\":";
        print_camera(camera);
        std::cout << ",\"imu\":";
        print_imu(imu);

        std::cout << ",\"gps\":";
        print_gps(gps_sample);
        std::cout << ",\"imu_window\":[";
        for (std::size_t index = 0; index < window.size(); ++index) {
            if (index != 0) {
                std::cout << ',';
            }
            print_imu(&window[index]);
        }
        std::cout << "]}\n";
    }

    void flush_ready   () {
        while (true) {
            auto ready_button = std::min_element(
                buttons.begin(), buttons.end(), [this](const Record& left, const Record& right) {
                    if (ready(left) != ready(right)) {
                        return ready(left);
                    }
                    return before(left, right);
                });
            if (ready_button == buttons.end() || !ready(*ready_button)) {
                return;
            }
            emit(*ready_button);
            buttons.erase(ready_button);
        }
    }

    void flush_all() {
        std::sort(buttons.begin(), buttons.end(), before);
        for (const Record& button : buttons) {
            emit(button);
        }
        buttons.clear();
    }

    void prune() {
        if (!has_timestamp) {
            return;
        }
        const uint64_t keep_from = newest_timestamp > gps_tolerance + reorder_tolerance
                                       ? newest_timestamp - gps_tolerance - reorder_tolerance
                                       : 0;
        auto remove_old = [keep_from](std::deque<Record>& samples) {
            samples.erase(std::remove_if(samples.begin(), samples.end(),
                                         [keep_from](const Record& sample) {
                                             return sample.timestamp < keep_from;
                                         }),
                           samples.end());
        };
        remove_old(cameras);
        remove_old(imus);
        remove_old(gps);
    }
};
} // namespace

ThreadWorker::ThreadWorker(std::size_t capacity) : capacity_(capacity) {}

void ThreadWorker::push(const Record& record) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [this] {
        return queue_.size() < capacity_ || finished_;
    });
    if (finished_) {
        return;
    }
    queue_.push(record);
    not_empty_.notify_one();
}

bool ThreadWorker::pop(Record& record) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] {
        return !queue_.empty() || finished_;
    });
    if (queue_.empty()) {
        return false;
    }
    record = queue_.front();
    queue_.pop();
    not_full_.notify_one();
    return true;
}

void ThreadWorker::finish() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        finished_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
}

bool ThreadWorker::input_failed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return input_failed_;
}

void ThreadWorker::set_input_failed() {
    std::lock_guard<std::mutex> lock(mutex_);
    input_failed_ = true;
}

uint16_t ThreadWorker::load_u16(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8);
}

uint32_t ThreadWorker::load_u32(const uint8_t* bytes) {
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8) |
           (static_cast<uint32_t>(bytes[2]) << 16) |
           (static_cast<uint32_t>(bytes[3]) << 24);
}

uint64_t ThreadWorker::load_u64(const uint8_t* bytes) {
    uint64_t value = 0;
    for (int index = 7; index >= 0; --index) {
        value = (value << 8) | bytes[index];
    }
    return value;
}

uint32_t ThreadWorker::fnv1a(const uint8_t* bytes, std::size_t size) {
    uint32_t hash = 2166136261u;
    for (std::size_t index = 0; index < size; ++index) {
        hash ^= bytes[index];
        hash *= 16777619u;
    }
    return hash;
}

bool ThreadWorker::read_header() {
    std::array<uint8_t, 16> bytes{};
    std::cin.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
    if (std::cin.gcount() != static_cast<std::streamsize>(bytes.size()) ||
        std::string(reinterpret_cast<char*>(bytes.data()), 8) != "USENS001" ||
        load_u16(bytes.data() + 8) != 32 || load_u16(bytes.data() + 10) != 1 ||
        fnv1a(bytes.data(), 12) != load_u32(bytes.data() + 12)) {
        set_input_failed();
        return false;
    }
    return true;
}

ThreadWorker::ReadResult ThreadWorker::read_record(Record& record) {
    std::array<uint8_t, 32> bytes{};
    std::cin.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
    const std::streamsize count = std::cin.gcount();
    if (count == 0) {
        return ReadResult::end;
    }
    if (count != static_cast<std::streamsize>(bytes.size())) {
        return ReadResult::error;
    }
    if (bytes[14] != 0 || bytes[15] != 0 || fnv1a(bytes.data(), 28) != load_u32(bytes.data() + 28)) {
        return ReadResult::invalid;
    }

    record.timestamp = load_u64(bytes.data());
    record.sequence = load_u32(bytes.data() + 8);
    record.sensor_id = bytes[12];
    record.flags = bytes[13];
    record.x = static_cast<int32_t>(load_u32(bytes.data() + 16));
    record.y = static_cast<int32_t>(load_u32(bytes.data() + 20));
    record.z = static_cast<int32_t>(load_u32(bytes.data() + 24));
    record.checksum = load_u32(bytes.data() + 28);
    return ReadResult::valid;
}

void ThreadWorker::reader() {
    if (read_header()) {
        Record record{};
        while (true) {
            const ReadResult result = read_record(record);
            if (result == ReadResult::end) {
                break;
            }
            if (result == ReadResult::error) {
                set_input_failed();
                break;
            }
            if (result == ReadResult::valid && (record.flags & valid_flag) != 0) {
                push(record);
            }
        }
    }
    finish();
}

void ThreadWorker::parser() {
    Synchronizer synchronizer;
    Record record{};
    while (pop(record)) {
        synchronizer.add(record);
        synchronizer.flush_ready();
        synchronizer.prune();
    }
    synchronizer.flush_all();
}
