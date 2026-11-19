"""Join the independent 1 kHz probe's returned FRMW with the HPM trace."""

import argparse
import csv
from pathlib import Path
import re
from statistics import median
import struct


def delta32(a, b):
    return (a - b + 2**31) % 2**32 - 2**31


def ring(path, words):
    data = path.read_bytes()
    capacity = 128
    if len(data) != 4 + capacity * words * 4:
        raise ValueError(f"{path}: unexpected trace ABI")
    count, = struct.unpack_from("<I", data)
    rows = list(struct.iter_unpack(f"<{words}I", data[4:]))
    return {r[0]: r for i in range(max(0, count-capacity), count) if (r := rows[i % capacity])[0]}


def sof_lead(row, start, period, shift):
    due = row["due_ns"]
    if (start - due) % period != shift:
        raise ValueError("DC register 0x0990 disagrees with the probe phase")
    # One frame between successive snapshots, SM2 returned, and FRMW received.
    # Minimum L2 length: Ethernet(14), EC header(2), FRMW(16), 784-byte LWR(796).
    if (row["sm2_frames"] != 1 or not 828 <= row["sm2_frame_bytes"] <= 1514 or
            row["output_wkc"] != 1 or row["dc_sof_valid"] != 1):
        raise ValueError("no same-frame proof for this SM2 and FRMW")
    lead = delta32(due + shift, row["dc_sof_ns"])
    if abs(lead) >= period // 2:
        raise ValueError("ambiguous DC frame epoch")
    return lead


def joined(directory):
    log = (directory / "counter-long.log").read_text()

    def register(address):
        return int(re.search(rf"^DC register {address} (0x[0-9a-fA-F]+)", log, re.M)[1], 16)

    start, period = register("0x0990"), register("0x09a0")
    shift = 1000 * int(re.search(r"^ONE_CYCLE Sync0_us=(\d+)", log, re.M)[1])
    if period != 1_000_000 or "DC system time transmission delay: 0 ns" not in (directory / "preflight.txt").read_text():
        raise ValueError("requires a single zero-delay HPM reference and a 1 ms cycle")
    rx = ring(directory / "long-rx.bin", 7)
    phase = ring(directory / "long-phase.bin", 18)
    diag = struct.unpack("<19I", (directory / "long-diag.bin").read_bytes())
    with (directory / "counter-long.csv").open(newline="") as f:
        host = [{k: int(v) for k, v in row.items()} for row in csv.DictReader(f)]
    for i, row in enumerate(host):
        seq, due = row["sent"], row["due_ns"]
        if i and (seq != host[i-1]["sent"] + 1 or due - host[i-1]["due_ns"] != period):
            raise ValueError("non-contiguous host trace")
        hw = sof_lead(row, start, period, shift)
        result = dict(seq=seq, ack=row["ack"], on_time=row["on_time"], l2_bytes=row["sm2_frame_bytes"],
                      host_send_us=(row["write_ns"] - due) / 1000, sof_lead_us=hw / 1000,
                      pdi_lead_us=None, sof_to_pdi_minus_sync_irq_us=None, esc_read_us=None,
                      commit_end_rel_sync_us=None)
        if seq in rx:
            x, ph = rx[seq], phase.get(seq)
            if row["on_time"] and ph and seq == ph[4] == row["ack"]:
                sync = ph[3]
                if not 0 <= delta32(ph[7], due + shift) < period:
                    raise ValueError(f"MCU DC anchor belongs to another cycle: {seq}")
            elif (not row["on_time"] and i and
                  diag[4] & 1 and diag[15] == 1 and seq-1 in phase and
                  phase[seq-1][4] == seq-1 and diag[12] == phase[seq-1][1] + 1 and
                  x[4] == diag[16]):
                sync = diag[13]
            else:
                raise ValueError(f"no proven MCU latch for {seq}")
            if not x[6] or abs(delta32(sync, x[1])) >= period // 2000:
                raise ValueError(f"stale or indirect SM2 stage: {seq}")
            sw = delta32(sync, x[1])
            result.update(pdi_lead_us=sw, sof_to_pdi_minus_sync_irq_us=hw/1000 - sw,
                          esc_read_us=delta32(x[3], x[2]), commit_end_rel_sync_us=delta32(x[5], sync))
        yield result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    rows = list(joined(args.directory))
    if args.csv:
        with args.csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0])
            writer.writeheader()
            writer.writerows(rows)
    print(f"Same-frame SOF samples: {len(rows)}, PDO passed: {sum(r['on_time'] for r in rows)}")
    matched = [r for r in rows if r["pdi_lead_us"] is not None and r["on_time"]]
    fields = ("sof_lead_us", "pdi_lead_us", "sof_to_pdi_minus_sync_irq_us", "esc_read_us", "host_send_us")
    print("seq-range", *fields)
    for i in range(0, len(matched), 32):
        group = matched[i:i+32]
        print(f"{group[0]['seq']}..{group[-1]['seq']}", *(f"{median(r[k] for r in group):.3f}" for k in fields))
    for r in rows[-2:]:
        print(r)


if __name__ == "__main__":
    main()
