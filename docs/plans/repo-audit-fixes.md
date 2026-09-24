# Plan: Repo audit fixes

## Context

A repo-wide audit (dead code, simplification, performance, security) found a
small number of concrete issues across `tools/alliance-planner/index.html`,
`core/`, `TeamCode/`, and docs. This plan fixes them. No spec document exists
for this repo beyond `docs/architecture.md` (design conventions) and the
audit findings themselves, which are the binding authority here.

## Global Constraints

- `tools/alliance-planner/index.html` has no build step and no test
  framework. Its "tests" are: `node --check` for syntax validity of the
  extracted `<script>` block, plus a `grep`-based assertion that the specific
  bug is fixed. Do not add a build step or test framework — out of scope.
- `core/` has a real JUnit suite (`./gradlew :core:test`, 301 tests, all
  currently passing). Any change there must keep that suite green and must
  not alter observable behavior (this is a pure performance change).
- Do not touch the easter-egg code in `tools/alliance-planner/index.html`
  (obfuscated titles / hidden flourishes) — leave it untouched and do not
  describe it in commits.
- Each task commits independently with a message describing that task's fix
  only.

## Pre-flight

Tasks share one file (`index.html`) in several places but touch disjoint
line ranges and disjoint functions; Task 6 (core/BezierPath) is fully
independent. No task's output is consumed as input by a later task's code —
each is an independent, self-contained fix. No shared-interface rows.

## Task 1: Fix XSS in obstacle-name rendering (alliance-planner)

**Files:** `tools/alliance-planner/index.html`

**Problem:** obstacle name is free user text, interpolated unescaped into
`innerHTML` in two places — `renderObstacles` (`value="${o.name}"`) and
`updateVerdict()` (a `<b>${n}</b>` template literal). A crafted name like
`"><img src=x onerror=alert(1)>` executes on render.

### Steps

1. Add a small `escapeHtml(s)` helper near the top of the `<script>` block
   (escape `&`, `<`, `>`, `"`, `'`).
2. **Test first (RED):** write a throwaway Node check that loads the escaped
   fragment logic and asserts a `"` and `<` in input produce no raw `<`/`"`
   in output. Concretely: extract `escapeHtml` into a temp `.mjs` alongside
   the check, or inline both in one script under `/tmp`, and run it — it
   must fail before the helper exists (helper doesn't exist yet → throws).
   Expected: script exits non-zero / throws `ReferenceError: escapeHtml is
   not defined`.
3. Implement `escapeHtml` (GREEN). Re-run the same check.
   Expected: exits 0, printed assertion output confirms `<script>` becomes
   `&lt;script&gt;` (or similar) with no unescaped `<`/`"` surviving.
4. Use `escapeHtml(o.name)` at the `renderObstacles` obstacle-name
   interpolation site, and at the `updateVerdict()` site where obstacle names
   are joined into `el.innerHTML`.
5. Verify with a repo-level check (not a full test suite, since none exists):
   ```
   node --check <(sed -n '/<script>/,/<\/script>/p' tools/alliance-planner/index.html | sed '1d;$d')
   ```
   Expected: no syntax error (exit 0).
6. Grep-verify both call sites now route through `escapeHtml`:
   ```
   grep -n 'escapeHtml(o.name)' tools/alliance-planner/index.html
   ```
   Expected: two matches (one per site).

### Commit

`Escape obstacle names before interpolating into innerHTML (XSS fix)`

## Task 2: Fix focus-loss bug on cap/wait field edits (alliance-planner)

**Files:** `tools/alliance-planner/index.html`

**Problem:** typing in the `cap`/`wait` number inputs fires `syncPanel()` on
every `input` event, which calls the full `renderSegments()` rebuild
(`innerHTML = ""` + re-append), destroying and recreating the very `<input>`
being typed into and losing cursor position mid-keystroke. The code already
distinguishes `structural` edits from non-structural ones near this handler;
non-structural edits should not trigger a full rebuild.

### Steps

1. **Reproduce first (RED):** confirm the bug exists by reading the current
   `cap`/`wait` input handler and tracing that it unconditionally calls
   `syncPanel()` → `renderSegments()` regardless of the `structural` flag.
   Record the exact current behavior as the "before" state (this stands in
   for a failing test, since the file has no test harness to automate this
   against a headless DOM).
2. Change the handler so a `cap`/`wait` edit updates the underlying data
   model and calls `syncPanel()`'s non-rendering parts only (or skips the
   `renderSegments()` call) when the edit is not `structural`; structural
   edits (`del`/`ctrl`/`mode`) keep triggering the full rebuild as before.
3. **Verify (GREEN):**
   ```
   node --check <(sed -n '/<script>/,/<\/script>/p' tools/alliance-planner/index.html | sed '1d;$d')
   ```
   Expected: exit 0.
4. Grep-verify the structural gate now wraps the rebuild call at this site:
   ```
   grep -n 'structural' tools/alliance-planner/index.html
   ```
   Expected: the cap/wait handler's rebuild call is now conditioned on
   `structural` (inspect the matched lines).
5. Manual smoke check (since this is a UI focus bug): open
   `tools/alliance-planner/index.html` in a browser (or note in the commit
   that this needs a manual check) and confirm typing multiple digits into a
   `cap` field does not lose focus. If a browser isn't available in this
   environment, note that in the ledger as a deferred manual-verification
   item rather than claiming it was checked.

### Commit

`Stop full segment-list rebuild on non-structural field edits`

## Task 3: Debounce collision scanning (alliance-planner)

**Files:** `tools/alliance-planner/index.html`

**Problem:** `invalidate()` fires on nearly every keystroke and triggers a
full `scanCollisions()` sweep (`duration/0.02s` steps) every time.

### Steps

1. Wrap the `scanCollisions()` call inside `invalidate()` (or `invalidate()`
   itself) in a ~120ms debounce using a module-level timer handle.
2. Verify:
   ```
   node --check <(sed -n '/<script>/,/<\/script>/p' tools/alliance-planner/index.html | sed '1d;$d')
   ```
   Expected: exit 0.
3. Grep-verify a `setTimeout`/debounce guard now wraps the scan call:
   ```
   grep -n 'scanCollisions' tools/alliance-planner/index.html
   ```
   Expected: call site no longer invoked synchronously and unconditionally
   from every keystroke path — inspect to confirm a timer sits between them.

### Commit

`Debounce collision scanning instead of running it on every keystroke`

## Task 4: Throttle canvas pointermove rendering (alliance-planner)

**Files:** `tools/alliance-planner/index.html`

**Problem:** `pointermove` calls `render()` synchronously on every event with
no rAF coalescing.

### Steps

1. Wrap the `render()` call inside the `pointermove` handler in a
   `requestAnimationFrame` coalescing guard (a pending-frame flag that
   collapses bursts of pointermove events into one render per frame).
2. Verify:
   ```
   node --check <(sed -n '/<script>/,/<\/script>/p' tools/alliance-planner/index.html | sed '1d;$d')
   ```
   Expected: exit 0.
3. Grep-verify:
   ```
   grep -n 'requestAnimationFrame' tools/alliance-planner/index.html
   ```
   Expected: a new match near the `pointermove` handler.

### Commit

`Coalesce pointermove canvas redraws with requestAnimationFrame`

## Task 5: Cleanup pass (alliance-planner)

**Files:** `tools/alliance-planner/index.html`

**Problem (grouped, all Minor readability items from the audit):**
- Orphaned JSDoc block sits above `rotatePath` but describes `movePoint`.
- `showText()` uses inline `style.cssText` instead of the app's CSS class
  convention.

### Steps

1. Move the misplaced JSDoc comment to sit directly above `movePoint`
   instead of `rotatePath`.
2. Replace `showText()`'s inline `style.cssText` with a CSS class added to
   the existing `<style>` block, applied via `classList.add`.
3. Verify:
   ```
   node --check <(sed -n '/<script>/,/<\/script>/p' tools/alliance-planner/index.html | sed '1d;$d')
   ```
   Expected: exit 0.

### Commit

`Fix misplaced JSDoc comment and move showText() styling into CSS`

## Task 6: Speed up BezierPath.getClosestT (core)

**Files:** `core/src/main/java/com/platapus/pathing/paths/BezierPath.java`,
`core/src/test/java/com/platapus/pathing/...` (existing tests must stay
green; add one perf/behavior-preserving test if the plan below introduces a
new code path worth covering).

**Problem:** `getClosestT` does a full 201-sample coarse scan every call
even when the caller supplies a good `tGuess` (the common case: every
control-loop call from `PathChain.project`). `deCasteljau` also allocates a
fresh scratch array on every evaluation.

### Steps

1. **RED:** run the existing suite to confirm current green baseline:
   ```
   ./gradlew :core:test
   ```
   Expected: BUILD SUCCESSFUL, all tests pass (this is the baseline, not a
   failing test — this change is a refactor under an existing test net, per
   TDD-for-refactors: characterize current behavior with the existing suite
   before changing anything).
2. Add a focused test in `GeometryTest.java` or `BezierPathTest.java`
   (whichever holds `getClosestT` coverage) that asserts `getClosestT` with a
   close, correct `tGuess` returns the same result (within existing
   tolerance) as with no guess / a poor guess, on both a simple curve and a
   self-crossing curve. If such a test already exists, confirm it and skip
   adding a duplicate — note that in the ledger instead of adding one.
3. Implement the local-search-first strategy in `getClosestT`: when
   `tGuess` is within `[0,1]` and not exactly at a boundary with no prior
   history, search a narrow window around it first (e.g. ±5 samples of the
   existing step) and only fall back to the full 201-sample scan if the
   local search doesn't find something competitive with what a full scan
   would (use the existing tie-breaking logic's own distance comparison to
   decide "competitive" — do not invent a new threshold beyond what the
   method already uses for the guess-vs-bestT comparison).
4. Give `deCasteljau`'s scratch array reuse: add an instance-level scratch
   buffer sized once (in the constructor, to `controlPoints.size()` and to
   the two derivative-point list sizes) and reuse it across
   `getPoint`/`getDerivative`/`getSecondDerivative` calls instead of
   allocating a new array per call.
5. **GREEN:** run the full suite again:
   ```
   ./gradlew :core:test
   ```
   Expected: BUILD SUCCESSFUL, same pass count as the baseline in step 1,
   including the new test from step 2.

### Commit

`Speed up BezierPath.getClosestT with a local search and reused scratch buffer`

## Task 7: Doc and hygiene fixes

**Files:** `README.md`, `docs/architecture.md`,
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ExampleTunedAutoOpMode.java`

### Steps

1. `README.md`: delete the duplicate "Tune feedforward before PID" paragraph
   (the second, shorter occurrence right before the "Phase 3 will fit..."
   line), and change "Phase 3 will fit `kS`, `kV` and `kA` online..." to
   present tense, consistent with the Roadmap section's "Phase 3 ... *Done.*"
2. `docs/architecture.md`: update the "234 tests, all headless" line to the
   current count. Get the current count with:
   ```
   grep -rn "@Test" core/src/test | wc -l
   ```
   and use that number (or reword to avoid a specific count that will drift
   again — implementer's call, ledger which was chosen).
3. `TeamCode/.../ExampleTunedAutoOpMode.java`: add
   `import com.platapus.pathing.tuning.StallListener;` to the import block
   and change the fully-qualified
   `new com.platapus.pathing.tuning.StallListener() { ... }` to
   `new StallListener() { ... }`.
4. Verify the Java change compiles. `:TeamCode` needs an Android SDK, which
   may not be present in this environment — if `ANDROID_HOME` /
   `local.properties` isn't set, verify by inspection only (the change is an
   import + qualified-name removal, mechanically safe) and note in the
   ledger that compilation wasn't verified by a build due to missing SDK.
   If an SDK is present:
   ```
   ./gradlew :TeamCode:compileDebugJavaWithJavac
   ```
   Expected: BUILD SUCCESSFUL.
5. Verify the README/docs edits with a grep check that the stale strings are
   gone:
   ```
   grep -n "Phase 3 will fit" README.md
   grep -n "234 tests" docs/architecture.md
   ```
   Expected: no matches (empty grep output) for both.

### Commit

`Fix stale docs (duplicate paragraph, future-tense Phase 3, stale test count) and missing import`

## Completion contract (whole plan)

- `./gradlew :core:test` passes with the same or greater test count than the
  Task 6 baseline.
- `node --check` on the extracted `<script>` block of
  `tools/alliance-planner/index.html` passes after every alliance-planner
  task.
- No easter-egg code was touched or described.
- Every deviation from this plan is ledgered as a `Ruling:`.
