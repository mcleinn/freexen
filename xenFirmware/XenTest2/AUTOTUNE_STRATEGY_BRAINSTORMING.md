# Autotune Strategy Brainstorming (Idle Stability)

Goal: deterministically reach `idleX total=0` under full-range scan (280 keys), within a bounded time budget, by tuning:
- global scan knobs: ADC averaging (`avg`) and mux settle delay (`sd_us`)
- per-key thresholds (`thr[key]`), RAM-only unless explicitly saved

Key constraints
- We prefer the fastest stable configuration.
- We want to raise per-key thresholds as little as needed, as much as necessary.
- The algorithm must be deterministic and converge. If it cannot reach zero offenders with thresholds up to a cap, that indicates a bug or a hardware condition that must be reported explicitly.

Observations from current firmware behavior
- Short windows can show `idle total==0` while longer windows (`idle60+`) still find rare spikes.
- Averaging changes tend to reduce totals quickly (global noise suppression).
- `sd_us` affects carryover/settling; too-low can create rare excursions.
- A threshold stepper is slow; one-shot threshold setting based on measured max excursion converges faster.

Definitions
- `exc[key] = abs(v - zeroV[key])`
- `maxExc[key] = max(exc[key])` over an observation window
- `excess[key] = maxExc[key] - thr[key]`
- `worstExcess = max_k(excess[key])`

Proposed staged strategy

Stage A: coarse global search (fast)
- Reset thresholds to calibrated baseline (`thrOrig`) at the start of each candidate evaluation.
- Evaluate a broad grid of (avg, sd_us) with a short window (e.g. 3–5s):
  - compute `worstExcess` and `totalCrossings`
- Rank candidates by:
  1) lowest `worstExcess` (distance-to-failure)
  2) lowest `totalCrossings`
  3) fastest (lower avg, lower sd_us) as tie-break
- Keep top K candidates.

Stage B: candidate refinement (medium)
- For each of top K candidates:
  - run a 15–30s window recording `maxExc[key]` and `counts[key]`
  - apply one-shot threshold update:
    `thr[key] = max(thr[key], maxExc[key] + margin)` for keys where `maxExc>thr`
  - re-run the same medium window once; require `totalCrossings==0`
- Record how many keys were raised and max threshold after raising.
- Prefer candidates that achieve 0 with fewer/smaller raises.

Stage C: strict validation ladder (rare-event catch)
- For best candidate:
  - run `idle60`; if pass, run `idle90`, then `idle120`.
  - if fail at a rung:
    - apply targeted update using max excursions observed during that rung (or an immediate 30s focused re-measure) for offending keys
    - repeat the rung
- Stop once the top rung passes, or a hard time budget is reached.

Determinism & guarantees
- This converges because thresholds only ever increase (monotonic).
- If threshold cap is sufficiently high, any finite observed max excursion can be suppressed.
- If we fail to converge while thresholds are increasing, it suggests:
  - measurement bug (e.g. counts not matching excursions)
  - baseline/key indexing issue
  - or a key stuck/noisy beyond cap; must be reported.

Logging requirements (RAM-only; dump later)
- Store a single run in RAM:
  - run header (range, time, parameters tried)
  - per-candidate summary (avg, sd_us, short window metrics)
  - per-pass summaries (window seconds, totals, worstExcess)
  - per-key details for interesting keys:
    - offenders at any stage
    - keys whose thresholds were raised
    - keys hitting threshold cap
- Provide a `autodump` command to dump the last run as JSONL.
