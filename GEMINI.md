# Eng-Expo Agent Rules

## Memory & Token Conservation (20-30% Reduction Target)

- When retrieving memory context, limit to the **3 most relevant nodes** (reduced from standard 5). Do not dump the entire graph.
- Summarize retrieved memory into **≤150 tokens** before injecting into working context to achieve strict token savings.
- Prefer `search_nodes` with a highly specific query over `read_graph` (which returns everything).
- When storing new memory, use extremely concise atomic observations (one short fact per observation). Avoid conversational fluff.
- Prune stale entities proactively: if a component or pin mapping has been superseded, delete the old entity immediately.

## Project Context (for memory seeding)

- Board: ESP32-DevKitC (espressif32, arduino framework)
- Display: ST7789 240×240 TFT (SPI: MOSI=23, SCLK=18, CS=5, DC=2, RST=4)
- Libraries: TFT_eSPI, ESP32Servo, HUSKYLENS, InEngMotor
- Monitor baud: 115200
- C++ standard: gnu++17
