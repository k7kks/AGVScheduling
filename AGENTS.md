# AGENTS.md

## Project overview
- C++17 AGV cluster scheduling and path planning system built with CMake.
- Primary binaries are built into `build/bin/` (e.g., `external_receiver`, `external_task_sender`).
- Uses RabbitMQ C client (`librabbitmq`) at link time.

## Build
```bash
cmake -S . -B build
cmake --build build -j
```

## Clean / reconfigure
```bash
cmake --build build --target distclean
cmake --build build --target reconfigure
```

## Tests
- No automated tests found.

## Notes
- If CMake errors about missing `librabbitmq`, install the RabbitMQ C client development package for your OS.
