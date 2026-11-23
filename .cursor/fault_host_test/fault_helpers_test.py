#!/usr/bin/env python3
"""Host-side behavioral tests for App_Fault helpers (mirrored logic)."""
from __future__ import annotations
import json
import time
from pathlib import Path

LOG = Path(__file__).resolve().parents[1] / "debug-c6be8a.log"
SESSION = "c6be8a"
fails = 0


def dlog(hyp: str, loc: str, msg: str, data: dict, run_id: str = "pre-refactor") -> None:
    rec = {
        "sessionId": SESSION,
        "runId": run_id,
        "hypothesisId": hyp,
        "location": loc,
        "message": msg,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with LOG.open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def expect(cond: bool, case: str) -> None:
    global fails
    if not cond:
        fails += 1
        print(f"FAIL: {case}")
        dlog("TEST", "expect", "fail", {"case": case})
    else:
        print(f"PASS: {case}")


# ---- mirrored helpers ----

def level(word: int, trip: int, rec: int, bit: int, hit: int, trip_n: int, rec_hit: int, rec_n: int):
    mask = 1 << bit
    if hit:
        rec = 0
        if trip < trip_n:
            trip += 1
        if trip >= trip_n:
            trip = trip_n
            word |= mask
    else:
        trip = 0
    if (hit == 0) and (rec_n != 0) and rec_hit and (word & mask):
        if rec < rec_n:
            rec += 1
        if rec >= rec_n:
            rec = 0
            word &= ~mask
    else:
        rec = 0
    return word & 0xFFFFFFFF, trip, rec


def level_simple(word: int, trip: int, rec: int, bit: int, hit: int, trip_n: int, rec_hit: int, rec_n: int):
    """Proposed simplified Level: early-return on hit (same observable state)."""
    mask = 1 << bit
    if hit:
        rec = 0
        if trip < trip_n:
            trip += 1
        if trip >= trip_n:
            trip = trip_n
            word |= mask
        return word & 0xFFFFFFFF, trip, rec
    trip = 0
    if (rec_n != 0) and rec_hit and (word & mask):
        if rec < rec_n:
            rec += 1
        if rec >= rec_n:
            rec = 0
            word &= ~mask
    else:
        rec = 0
    return word & 0xFFFFFFFF, trip, rec


def retry_lock(hits: int, win: int, alm: int, rising: int, hit_max: int, win_max: int, alm_mask: int):
    if rising:
        hits += 1
    if hits >= hit_max:
        alm |= alm_mask
        hits = 0
        win = 0
    elif hits != 0:
        win += 1
        if win >= win_max:
            hits = 0
            win = 0
    return hits, win, alm


def rec_cnt(c: int, bit_set: int, ok: int, rec_n: int):
    if bit_set == 0 or ok == 0:
        return 0, 0
    if c < rec_n:
        c += 1
    if c >= rec_n:
        return 0, 1
    return c, 0


def confirm(c: int, hit: int, n: int):
    if hit == 0:
        return 0, 0
    if c < n:
        c += 1
    return c, int(c >= n)


def imb_hit(i0: float, i1: float) -> int:
    s = abs(i0 + i1)
    d = abs(i0 - i1)
    lim = 0.30 * s
    if lim < 2.0:
        lim = 2.0
    return int(s > 2.0 and d > lim)


def main() -> int:
    global fails
    if LOG.exists():
        LOG.unlink()
    dlog("A", "main:entry", "host helper test start", {"suite": "fault_helpers"})

    # H-A trip
    w, t, r = 0, 0, 0
    for _ in range(3):
        w, t, r = level(w, t, r, 1, 1, 3, 1, 2)
    expect(w == 2 and t == 3, "level_trip_N")
    dlog("A", "Level:trip", "after 3 hits bit1 set", {"word": w, "trip": t})

    w, t, r = level(w, t, r, 1, 0, 3, 0, 2)
    expect(t == 0 and w == 2, "level_break_keeps_latch")

    # H-B recover
    w, t, r = level(w, t, r, 1, 0, 3, 1, 2)
    expect(w == 2 and r == 1, "level_rec_progress")
    w, t, r = level(w, t, r, 1, 0, 3, 1, 2)
    expect(w == 0 and r == 0, "level_rec_clear")
    dlog("B", "Level:rec", "recover cleared bit after 2", {"word": w})

    # H-C simplified equiv
    seq = [(1, 0), (1, 0), (1, 0), (0, 1), (0, 1), (1, 0), (0, 0), (0, 1), (0, 1), (0, 1)]
    w1 = t1 = r1 = w2 = t2 = r2 = 0
    match = True
    for hit, rh in seq:
        w1, t1, r1 = level(w1, t1, r1, 0, hit, 3, rh, 2)
        w2, t2, r2 = level_simple(w2, t2, r2, 0, hit, 3, rh, 2)
        if (w1, t1, r1) != (w2, t2, r2):
            match = False
    expect(match, "level_simple_equiv")
    dlog("C", "Level:equiv", "simple Level matches original", {"match": int(match)})

    # H-D retry lock
    hits = win = alm = 0
    hits, win, alm = retry_lock(hits, win, alm, 1, 3, 10, 1)
    hits, win, alm = retry_lock(hits, win, alm, 0, 3, 10, 1)
    hits, win, alm = retry_lock(hits, win, alm, 1, 3, 10, 1)
    hits, win, alm = retry_lock(hits, win, alm, 1, 3, 10, 1)
    expect(alm == 1 and hits == 0, "retry_lock_on_3")
    dlog("D", "RetryLock", "3rd rising sets alm", {"alm": alm})

    hits = win = alm = 0
    hits, win, alm = retry_lock(hits, win, alm, 1, 3, 3, 1)
    hits, win, alm = retry_lock(hits, win, alm, 0, 3, 3, 1)
    hits, win, alm = retry_lock(hits, win, alm, 0, 3, 3, 1)
    expect(alm == 0 and hits == 0 and win == 0, "retry_window_expire")

    # H-E RecCnt
    c, done = rec_cnt(0, 1, 1, 3)
    expect(done == 0 and c == 1, "rec_1")
    c, done = rec_cnt(c, 1, 1, 3)
    expect(done == 0 and c == 2, "rec_2")
    c, done = rec_cnt(c, 1, 1, 3)
    expect(done == 1 and c == 0, "rec_done")
    c, done = rec_cnt(c, 1, 0, 3)
    expect(done == 0 and c == 0, "rec_reset_on_not_ok")
    dlog("E", "RecCnt", "rec N then clear", {"ok": 1})

    # H-F Confirm
    c, out = confirm(0, 1, 3)
    expect(out == 0, "conf_1")
    c, out = confirm(c, 1, 3)
    expect(out == 0, "conf_2")
    c, out = confirm(c, 1, 3)
    expect(out == 1, "conf_3")
    c, out = confirm(c, 0, 3)
    expect(out == 0 and c == 0, "conf_break")

    # H-G Imb
    expect(imb_hit(5.0, 5.0) == 0, "imb_balanced")
    expect(imb_hit(10.0, 1.0) == 1, "imb_unbalanced")
    expect(imb_hit(0.5, -0.5) == 0, "imb_small_sum")
    dlog("G", "ImbHit", "imbalance formula checks", {"done": 1})

    # H-H SCI truncation hazard
    idle32 = 70000
    idle16 = idle32 & 0xFFFF  # 4464
    thr = 10000
    expect((idle16 >= thr) is False and (idle32 >= thr) is True, "sci_u16_truncation_hazard")
    dlog(
        "H",
        "SCI:trunc",
        "Uint16 trunc can miss SCI fault if idle>65535",
        {"idle32": idle32, "idle16": idle16, "thr": thr, "miss_if_u16": 1},
    )

    # Note: production caps idle at 60s so hazard is latent unless CAP raised
    dlog(
        "H2",
        "SCI:cap",
        "production UART_PROTO_IDLE_CAP_MS=60000 keeps Uint16 safe today",
        {"cap_ms": 60000, "fits_u16": 1},
    )

    dlog("Z", "main:exit", "host helper test done", {"pass": int(fails == 0)})
    print(f"\n{'FAILED' if fails else 'ALL PASS'} ({fails} failures)")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
