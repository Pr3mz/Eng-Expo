---
name: memory-best-practices
description: >-
  Enforces token-efficient memory usage patterns. Activate when storing
  or retrieving entities via MCP memory tools.
---

# Memory Best Practices

## Storing

1. One observation per `add_observations` call — atomic, concise facts only.
2. Use consistent entity names: `ESP32_Board`, `TFT_Display`, `HUSKYLENS_Sensor`, `Motor_Driver`.
3. Always specify `entityType` (HARDWARE, LIBRARY, CONFIG, BUG, DECISION).
4. Strip out unnecessary words to keep storage footprint small.

## Retrieving

1. Always use `search_nodes` with a targeted query before falling back to `read_graph`.
2. Limit context injection to ≤3 nodes and ≤150 tokens to enforce a 20-30% token reduction policy.
3. Summarize retrieved nodes; do not paste raw JSON into the response.

## Pruning

1. After resolving a bug, mark the bug entity's observation as resolved or delete it.
2. When a hardware pin mapping changes, delete the old observation and create a new one.
