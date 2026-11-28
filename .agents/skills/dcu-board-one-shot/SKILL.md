---
name: dcu-board-one-shot
description: Run one protected WG3 HPM/RK board acceptance shot for production 500 Hz, 3+5+5 motor PDO, or judge its saved evidence. Use for DCU single-shot all-zero MIT N+1 board verification; not for normal motion or endurance testing.
---

# DCU Board One Shot

Run from the `Dcu_dev` checkout. This skill loads the existing production firmware, RK DriverModule and their generated contract. It does not install or deploy RK software.

## Run On Hardware

1. Confirm the user authorized **one** all-zero MIT shot and physically isolated motor power. Confirm the RK SSH target and checkout and whether resetting/verifying or flashing HPM is authorized. All-zero MIT is a test command, not a stop mechanism.
2. Run in a dedicated local tmux pane. The runner starts only its own protected debug server and DriverModule; it stops if a previous owner is active. On the known WG3 bench the command is:

   ```bash
   python3 .agents/skills/dcu-board-one-shot/scripts/board_once.py run \
     --rk wg3_1 --rk-repo /home/zkbot/rob_dev/wg3_deploy --confirm-isolated
   ```

   Add `--build` to rebuild `hpm-release`. Add `--flash` only with flash authorization; otherwise the protected server verifies the already-installed image against the local ELF. A verification still resets or halts HPM. The verified RK runtime and ROS Humble environment must already exist.
3. Read the emitted evidence directory under `artifacts/target/`. Report the verdict, axes staged on time, each axis's TX counter delta, N+1 sequence/fresh mask, and strict SM3 completion time. PASS requires one publication, unchanged boot/layout identity, all 13 same-command causal feedback records, all TX deltas `+1`, timely SM3, and RK N+1 with the full fresh mask. A missing artifact or uncertain outcome is STOP, never PASS.

On failure, interruption, or loss of contact, keep the evidence and stop. Do not replay the shot, switch to a long run, or reconnect to reset the target to obtain a result. Verify Idle/PREOP before any further target work.

## Read Existing Evidence

```bash
python3 .agents/skills/dcu-board-one-shot/scripts/board_once.py report <evidence-dir>
```

For old runs without a saved layout, add `--layout <same-build-generated-layout.json>`. Report mode only reads local files. Exit codes: `0` PASS, `1` measured FAIL, `2` incomplete/STOP. Unrecorded HPM timestamps do not prove a missing CAN wire reply.
