# TestAP2 Autopilot Coding Agent

You are continuing development on the TestAP2 marine autopilot firmware.

## Session Startup Routine

1. Run `pwd` to confirm working directory is `/home/cas/TestAP2`
2. Run `git log --oneline -10` to see recent work
3. Read `docs/claude-progress.txt` for context
4. Read `agent/feature-list.json` to find next incomplete feature
5. Run `idf.py build` to verify existing code compiles
6. Only proceed if build succeeds

## Development Cycle

For each feature:

1. **Understand the requirement** - Read acceptance criteria carefully
2. **Read existing code** - Understand the codebase before modifying
3. **Write the implementation** - Minimal, focused changes
4. **Build** - `idf.py build`
5. **Mark complete** - Update feature-list.json `complete: true`
6. **Update progress** - Append summary to claude-progress.txt
7. **Commit** - Descriptive message referencing feature ID

## Safety Rules

- NEVER remove or weaken existing safety checks
- NEVER exceed rudder limit constants (±35°)
- NEVER bypass watchdog timers
- ALL sensor inputs must have timeout handling
- ALL control outputs must have range clamping

## Code Style

- Follow existing code patterns in the codebase
- Use ESP-IDF logging (`ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE`)
- All I2C access must use the global mutex
- Reference FSD sections in comments

## End of Session

Before stopping:
- Ensure code compiles without errors
- No new warnings introduced
- Progress file is updated
- Changes are committed
- Codebase is in a clean state for next session

## Key Files Reference

| File | Purpose |
|------|---------|
| `main/master_node.c` | Master node implementation |
| `main/rudder_node.c` | Rudder node implementation |
| `components/*/include/*.h` | Component headers |
| `docs/TestAP2.FSD.v1.0.0.md` | Authoritative specification |
