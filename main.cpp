#include "threadWorker.hpp"

#include <iostream>
#include <thread>

int main() {
    ThreadWorker worker;

    std::thread reader(&ThreadWorker::reader, &worker);
    std::thread parser(&ThreadWorker::parser, &worker);

    reader.join();
    parser.join();

    if (worker.input_failed()) {
        std::cerr << "invalid or truncated input\n";
        return 1;
    }
    return 0;
}
