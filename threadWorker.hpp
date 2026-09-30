#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <queue>

struct Record {
    uint64_t timestamp;
    uint32_t sequence;
    uint8_t sensor_id;
    uint8_t flags;
    int32_t x;
    int32_t y;
    int32_t z;
    uint32_t checksum;
};

class ThreadWorker {
public:
    explicit ThreadWorker(std::size_t capacity = 512);
    ThreadWorker(const ThreadWorker&) = delete;
    ThreadWorker& operator=(const ThreadWorker&) = delete;

    void reader();
    void parser();
    bool input_failed() const;

private:
    enum class ReadResult { end, valid, invalid, error };

    void push(const Record& record);
    bool pop(Record& record);
    void finish();
    void set_input_failed();

    bool read_header();
    ReadResult read_record(Record& record);
    static uint16_t load_u16(const uint8_t* bytes);
    static uint32_t load_u32(const uint8_t* bytes);
    static uint64_t load_u64(const uint8_t* bytes);
    static uint32_t fnv1a(const uint8_t* bytes, std::size_t size);

    std::queue<Record> queue_;
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    bool finished_ = false;
    bool input_failed_ = false;
};
