/**
 * Host-side behavioral tests for App_Fault helpers (mirrored logic).
 * Proves Level / RetryLock / RecCnt / Confirm / ImbHit semantics before refactor.
 * Writes NDJSON to debug-c6be8a.log for debug-mode evidence.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define LOG_PATH "E:/project-240908/haineng/10kw-dab/HybridInveter_DCDC_F28P550_APP_PLAIN_EXPORT/.cursor/debug-c6be8a.log"

typedef uint16_t Uint16;
typedef uint32_t Uint32;

typedef struct { Uint16 trip; Uint16 rec; } FaultTick_t;
typedef struct { Uint16 hits; Uint16 win; } RetryLock_t;

static FILE *g_log;

static void dlog(const char *hyp, const char *loc, const char *msg, const char *data)
{
    long long ts = (long long)time(NULL) * 1000LL;
    if (!g_log) return;
    fprintf(g_log,
        "{\"sessionId\":\"c6be8a\",\"runId\":\"pre-refactor\",\"hypothesisId\":\"%s\","
        "\"location\":\"%s\",\"message\":\"%s\",\"data\":%s,\"timestamp\":%lld}\n",
        hyp, loc, msg, data, ts);
    fflush(g_log);
}

/* ---- mirrored helpers (must match App_Fault.c semantics) ---- */

static void Fault_SetBit(volatile Uint32 *p, Uint32 m) { *p |= m; }
static void Fault_ClearBit(volatile Uint32 *p, Uint32 m) { *p &= ~m; }
static void Fault_SetBit16(volatile Uint16 *p, Uint16 m) { *p |= m; }

static Uint16 Fault_Confirm(Uint16 *c, Uint16 hit, Uint16 n)
{
    if (hit == 0U) { *c = 0U; return 0U; }
    if (*c < n) (*c)++;
    return (Uint16)(*c >= n);
}

static void Fault_Level(volatile Uint32 *pAll, FaultTick_t *pTick, Uint16 bit,
                        Uint16 hit, Uint16 tripN, Uint16 recHit, Uint16 recN)
{
    Uint32 mask = (Uint32)1UL << bit;
    if (hit != 0U) {
        pTick->rec = 0U;
        if (pTick->trip < tripN) pTick->trip++;
        if (pTick->trip >= tripN) {
            pTick->trip = tripN;
            Fault_SetBit(pAll, mask);
        }
    } else {
        pTick->trip = 0U;
    }
    if ((hit == 0U) && (recN != 0U) && (recHit != 0U) && ((*pAll & mask) != 0UL)) {
        if (pTick->rec < recN) pTick->rec++;
        if (pTick->rec >= recN) {
            pTick->rec = 0U;
            Fault_ClearBit(pAll, mask);
        }
    } else {
        pTick->rec = 0U;
    }
}

static void Fault_RetryLockTick(RetryLock_t *p, Uint16 rising,
                                Uint16 hitMax, Uint16 winMax, Uint16 almMask,
                                volatile Uint16 *alm)
{
    if (rising != 0U) p->hits++;
    if (p->hits >= hitMax) {
        Fault_SetBit16(alm, almMask);
        p->hits = 0U;
        p->win  = 0U;
    } else if ((p->hits != 0U) && (++p->win >= winMax)) {
        p->hits = 0U;
        p->win  = 0U;
    }
}

static Uint16 Fault_RecCntTick(Uint16 *c, Uint16 bitSet, Uint16 ok, Uint16 recN)
{
    if ((bitSet == 0U) || (ok == 0U)) { *c = 0U; return 0U; }
    if (*c < recN) (*c)++;
    if (*c >= recN) { *c = 0U; return 1U; }
    return 0U;
}

static float Fault_Abs(float x) { return (x >= 0.0f) ? x : -x; }

static Uint16 Fault_ImbHit(float i0, float i1)
{
    float sum = Fault_Abs(i0 + i1);
    float diff = Fault_Abs(i0 - i1);
    float lim = 0.30f * sum;
    if (lim < 2.0f) lim = 2.0f;
    return (Uint16)((sum > 2.0f) && (diff > lim));
}

/* ---- simplified Level candidate (proposed) ---- */
static void Fault_Level_Simple(volatile Uint32 *pAll, FaultTick_t *pTick, Uint16 bit,
                               Uint16 hit, Uint16 tripN, Uint16 recHit, Uint16 recN)
{
    Uint32 mask = (Uint32)1UL << bit;
    if (hit != 0U) {
        pTick->rec = 0U;
        if (pTick->trip < tripN) pTick->trip++;
        if (pTick->trip >= tripN) {
            pTick->trip = tripN;
            Fault_SetBit(pAll, mask);
        }
        return; /* hit: no recover path */
    }
    pTick->trip = 0U;
    if ((recN != 0U) && (recHit != 0U) && ((*pAll & mask) != 0UL)) {
        if (pTick->rec < recN) pTick->rec++;
        if (pTick->rec >= recN) {
            pTick->rec = 0U;
            Fault_ClearBit(pAll, mask);
        }
    } else {
        pTick->rec = 0U;
    }
}

static int g_fail;

static void expect(int cond, const char *json_case)
{
    if (!cond) {
        g_fail++;
        printf("FAIL: %s\n", json_case);
        dlog("TEST", "fault_helpers_test.c:expect", "fail", json_case);
    } else {
        printf("PASS: %s\n", json_case);
    }
}

int main(void)
{
    volatile Uint32 word = 0;
    FaultTick_t tick = {0};
    RetryLock_t lock = {0};
    volatile Uint16 alm = 0;
    Uint16 conf = 0;
    Uint16 rec = 0;
    int i;

    g_log = fopen(LOG_PATH, "a");
    dlog("A", "main:entry", "host helper test start", "{\"suite\":\"fault_helpers\"}");

    /* H-A: Level trip needs N consecutive hits */
    word = 0; tick.trip = 0; tick.rec = 0;
    for (i = 0; i < 3; i++)
        Fault_Level(&word, &tick, 1, 1, 3, 1, 2);
    expect(word == 2U && tick.trip == 3, "{\"case\":\"level_trip_N\"}");
    dlog("A", "Level:trip", "after 3 hits bit1 set",
         "{\"word\":2,\"trip\":3}");

    /* break continuity resets trip */
    Fault_Level(&word, &tick, 1, 0, 3, 0, 2); /* hit=0, recHit=0 → keep bit, reset trip */
    expect(tick.trip == 0 && word == 2U, "{\"case\":\"level_break_keeps_latch\"}");

    /* H-B: recover needs recN with recHit, and not while hit */
    tick.rec = 0;
    Fault_Level(&word, &tick, 1, 0, 3, 1, 2);
    expect(word == 2U && tick.rec == 1, "{\"case\":\"level_rec_progress\"}");
    Fault_Level(&word, &tick, 1, 0, 3, 1, 2);
    expect(word == 0U && tick.rec == 0, "{\"case\":\"level_rec_clear\"}");
    dlog("B", "Level:rec", "recover cleared bit after 2", "{\"word\":0}");

    /* H-C: simplified Level matches original across sequence */
    {
        volatile Uint32 w1 = 0, w2 = 0;
        FaultTick_t t1 = {0}, t2 = {0};
        Uint16 seq[][3] = {
            /* hit, recHit, pad */
            {1,0,0},{1,0,0},{1,0,0},{0,1,0},{0,1,0},
            {1,0,0},{0,0,0},{0,1,0},{0,1,0},{0,1,0}
        };
        int match = 1;
        for (i = 0; i < 10; i++) {
            Fault_Level(&w1, &t1, 0, seq[i][0], 3, seq[i][1], 2);
            Fault_Level_Simple(&w2, &t2, 0, seq[i][0], 3, seq[i][1], 2);
            if (w1 != w2 || t1.trip != t2.trip || t1.rec != t2.rec) match = 0;
        }
        expect(match, "{\"case\":\"level_simple_equiv\"}");
        dlog("C", "Level:equiv", "simple Level matches original",
             match ? "{\"match\":1}" : "{\"match\":0}");
    }

    /* H-D: RetryLock locks on 3rd rising within window */
    alm = 0; lock.hits = 0; lock.win = 0;
    Fault_RetryLockTick(&lock, 1, 3, 10, 1, &alm); /* hit1 */
    Fault_RetryLockTick(&lock, 0, 3, 10, 1, &alm);
    Fault_RetryLockTick(&lock, 1, 3, 10, 1, &alm); /* hit2 */
    Fault_RetryLockTick(&lock, 1, 3, 10, 1, &alm); /* hit3 → lock */
    expect(alm == 1 && lock.hits == 0, "{\"case\":\"retry_lock_on_3\"}");
    dlog("D", "RetryLock", "3rd rising sets alm", "{\"alm\":1}");

    /* window expiry resets without lock */
    alm = 0; lock.hits = 0; lock.win = 0;
    Fault_RetryLockTick(&lock, 1, 3, 3, 1, &alm);
    Fault_RetryLockTick(&lock, 0, 3, 3, 1, &alm);
    Fault_RetryLockTick(&lock, 0, 3, 3, 1, &alm); /* win reaches 3 → reset */
    expect(alm == 0 && lock.hits == 0 && lock.win == 0, "{\"case\":\"retry_window_expire\"}");

    /* H-E: RecCntTick */
    rec = 0;
    expect(Fault_RecCntTick(&rec, 1, 1, 3) == 0 && rec == 1, "{\"case\":\"rec_1\"}");
    expect(Fault_RecCntTick(&rec, 1, 1, 3) == 0 && rec == 2, "{\"case\":\"rec_2\"}");
    expect(Fault_RecCntTick(&rec, 1, 1, 3) == 1 && rec == 0, "{\"case\":\"rec_done\"}");
    expect(Fault_RecCntTick(&rec, 1, 0, 3) == 0 && rec == 0, "{\"case\":\"rec_reset_on_not_ok\"}");
    dlog("E", "RecCnt", "rec N then clear", "{\"ok\":1}");

    /* H-F: Confirm */
    conf = 0;
    expect(Fault_Confirm(&conf, 1, 3) == 0, "{\"case\":\"conf_1\"}");
    expect(Fault_Confirm(&conf, 1, 3) == 0, "{\"case\":\"conf_2\"}");
    expect(Fault_Confirm(&conf, 1, 3) == 1, "{\"case\":\"conf_3\"}");
    expect(Fault_Confirm(&conf, 0, 3) == 0 && conf == 0, "{\"case\":\"conf_break\"}");

    /* H-G: ImbHit */
    expect(Fault_ImbHit(5.0f, 5.0f) == 0, "{\"case\":\"imb_balanced\"}");
    expect(Fault_ImbHit(10.0f, 1.0f) == 1, "{\"case\":\"imb_unbalanced\"}");
    expect(Fault_ImbHit(0.5f, -0.5f) == 0, "{\"case\":\"imb_small_sum\"}");
    dlog("G", "ImbHit", "imbalance formula checks", "{\"done\":1}");

    /* H-H: SCI Uint16 truncation hazard simulation */
    {
        uint32_t idle32 = 70000U; /* >65535 */
        Uint16 idle16 = (Uint16)idle32; /* wraps */
        Uint16 thr = 10000U;
        int bad = (idle16 >= thr);   /* wrapped 70000→4464 → false, miss fault */
        int good = (idle32 >= thr);  /* true */
        expect(bad == 0 && good == 1, "{\"case\":\"sci_u16_truncation_hazard\"}");
        dlog("H", "SCI:trunc", "Uint16 trunc can miss SCI fault if idle>65535",
             "{\"idle32\":70000,\"idle16\":4464,\"thr\":10000,\"miss_if_u16\":1}");
    }

    dlog("Z", "main:exit", "host helper test done",
         g_fail ? "{\"pass\":0}" : "{\"pass\":1}");
    if (g_log) fclose(g_log);

    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
