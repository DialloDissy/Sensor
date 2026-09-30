# usensor

## Build

```sh
g++ -std=c++17 -O2 -Wall -Wextra -pedantic main.cpp threadWorker.cpp -pthread -o solution
```

## Run

The program reads the binary trace from standard input and writes one JSON line per valid button event.

```sh
./usensor --seed 42 --duration 300s | ./solution
```

The reader thread validates the header, reads records and pushes them into a bounded queue. The parser thread consumes the queue and keeps only the recent camera, IMU and GPS samples needed for synchronization. The queue and the sample history are bounded, and a condition variable avoids busy waiting.
