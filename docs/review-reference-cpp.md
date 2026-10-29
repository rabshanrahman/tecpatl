# Jackknife reference implementation (C++): clean-room findings

These notes come from the "dirty" side of a clean-room process. They contain prose and math only. No source text, identifiers, comments or file layout from the reference code is reproduced here. File and line references are given for provenance only; the implementer should not open those files.

Sources reviewed (cloned at depth 1 from the public ISUE repository; nothing from it was built or run):
- Core recognizer: cpp/jackknife/ (option struct; features; template; recognizer; training; math helpers; vector class; sample; smoothing filter)
- Evaluation harness and dataset loaders: cpp/evaluate/ (text loader for segmented training data; binary session reader and continuous-mode driver; user-independent test; confusion-matrix scoring)
- I spot-checked the JavaScript and C# ports only to see whether they share particular defects. Where I say they agree, I checked it.
- Dataset facts below come from a separate read-only probe script that I wrote and kept outside the clone. It parsed the data files and executed none of the repository's code.

Notation: n is the resample count, r is the band radius, m is the number of components per point, q is the query and t is a template. The inner product is written ⟨a,b⟩.

---

## 1. Adversarial QA

Each item gives where I found it, what happens, whether it is a real defect, and what the correct behavior should be.

### 1.1 Zero-length direction vectors become NaN, and NaN breaks sorting
- Where: features construction (jackknife_features.h ~l.115–137); vector normalization (vector.h ~l.360–372, which divides by the length without a guard).
- In inner-product (IP) mode, each direction vector is the difference of two consecutive resampled points divided by its own length. If two consecutive resampled points coincide, the result is 0/0, which is NaN in every component.
- When it happens:
  - The whole input has zero path length (a stationary input). The resampler then emits n copies of the same point (see 1.3), so every direction vector is NaN.
  - The path returns exactly to a previous position within one resampling interval. In several dimensions this has measure zero. In 1-D it is plausible, because a reversal inside one interval can give a zero net displacement.
- How the NaN spreads:
  - DTW: every cell that touches a NaN becomes NaN, and so does the final score. A NaN score is never "greater than the threshold" and never "less than the best", so that template is silently ignored and is not reported.
  - Lower bound: the bound becomes NaN. The templates are then sorted by lower bound with a plain less-than comparison. NaN violates strict weak ordering, which is undefined behavior in the standard sort. With more than about 16 templates, the introsort path runs (the Kinect session uses 28 templates), and some library implementations can then read out of bounds.
  - Correction factors: the clamp takes the larger of 0.01 and the dot product, with 0.01 as the first argument. A NaN dot product therefore yields exactly 0.01, because the comparison is false and the first argument is returned. The correction factor becomes the maximum penalty (100) rather than NaN. This is accidental behavior that depends on argument order.
- Verdict: real defect.
- Correct behavior:
  - Reject or skip any input whose path length is below an epsilon.
  - When a step has zero length, use a defined fallback: either drop that direction (and keep counts consistent) or reuse the previous direction.
  - Never feed NaN into an ordering.
  - Clamp correction factors explicitly and treat a zero vector as "no information" (factor 1), or reject the input.

### 1.2 Z-score normalization with zero variance
- Where: math helpers (mathematics.h ~l.35–83).
- Behavior:
  - The mean is taken per component.
  - The variance uses the unbiased divisor n−1.
  - Each component is divided by its standard deviation, with no guard.
- Problems:
  - Any component that is constant across the resampled sequence gives 0/0 = NaN, or ±inf if the value deviates by a rounding error. This happens in ED mode, where it is on by default: for example, a planar gesture in 3-D, or an unused joint coordinate in a 63-dimensional skeleton.
  - With a single point, the n−1 divisor is zero.
- Verdict: real defect.
- Correct behavior: if the standard deviation is below an epsilon, set that component to 0 (or leave it centered but unscaled).

### 1.3 Resampling edge cases
- Where: math helpers (mathematics.h ~l.109–207).
- What it does:
  - It computes the total polyline length L.
  - It places n points at equal arc-length spacing L/(n−1). The first output point is the first raw point.
  - It walks the raw segments. While the current segment is shorter than the remaining interval distance, it consumes the segment. Otherwise it linearly interpolates at the fraction (remaining ÷ segment length) and emits that point. The emitted point becomes the new "previous" point, and the same raw segment is examined again.
  - If the walk ends with fewer than n points (the usual case, from floating-point shortfall), it appends the last raw point.
  - Interpolation is (1−t)·a + t·b.
  - The fraction is clamped to 1 if it exceeds 1 or is NaN.
- Hazards:
  - **Input with one point:** the loop never runs, and only two points are produced. The debug assertion that the count equals n fails. In release builds, the code downstream indexes past the end, which is undefined behavior. An empty input is undefined behavior immediately, because the first point is read unconditionally.
  - **n = 1:** the spacing is 1/0 = inf. The interval array has length 0 but its first element is read, which is out of bounds. In IP mode, n = 1 also gives zero direction vectors, and later code reads the first vector unconditionally.
  - **n = 2 (IP):** this gives one direction vector, the chord. It is legal but degenerate.
  - **Stationary input (L = 0):** every comparison is 0 < 0, which is false. The fraction is NaN, clamped to 1. The resampler emits n identical points and terminates. The result is the NaN cascade in 1.1 and 1.2.
  - **Output container:** the resampler assumes the output container starts empty and holds a pointer into it (safe only because capacity is reserved up front). If a caller passes a non-empty container, the counts break.
  - **Shortfall of more than one point:** the post-loop fix adds only one point. If accumulated rounding left more than one point missing, the count would be wrong. This is not observed for n = 16, but it is not guaranteed.
  - **Inputs shorter than n:** these are fine, because interpolation creates the extra points.
- Correct behavior:
  - Require at least two distinct points, and positive length.
  - Compute the spacing in double precision.
  - Pad by repeating the last point until there are exactly n points.
  - Pin the final output to the last raw point.

### 1.4 The option struct is packed into 8-bit bitfields and is uninitialized by default
- Where: option struct (jackknife.h ~l.31–112).
- Resample count and radius are 8-bit unsigned bitfields. Values of 256 or more wrap modulo 256; for example, n = 256 becomes 0, which then triggers the n = 0/1 failures above.
- The struct has no constructor. Unless the caller invokes one of the two "set defaults" routines, every field holds garbage.
- The recognizer's constructor only asserts that exactly one of the two measures (IP or ED) is selected.
- Nothing prevents incoherent combinations, for example z-normalization together with IP.
- Correct behavior:
  - Use full-width integers with validation: n ≥ 2 (for IP, n ≥ 3 is sensible), 0 ≤ r < number of vectors.
  - Initialize defaults explicitly.
  - Reject illegal combinations.

### 1.5 Z-normalization with IP makes the cost and the lower bound invalid
- Where: features (jackknife_features.h ~l.158–161); the IP cost (jackknife_recognizer.h ~l.89–92).
- If z-normalization is enabled in IP mode, the direction vectors are rescaled per component after unit normalization, so they are no longer unit vectors. Then:
  - The local cost 1 − ⟨q,t⟩ can be negative, so DTW is no longer a sum of non-negative terms.
  - The lower bound's clip to [−1, 1] is no longer a valid bound.
  - Lower-bound pruning can then wrongly discard the true best template.
- The comments advise against this combination, but the code does not forbid it.
- Correct behavior: forbid the combination, or re-normalize to unit length after any per-component scaling.

### 1.6 The rejection threshold is learned without correction factors but applied with them
- Where:
  - Training: jackknife_train.cpp ~l.201–203 (negatives) and ~l.255–257 (positives) score with plain DTW.
  - Classification: jackknife_recognizer.h ~l.241–301 multiplies DTW by the correction factor before comparing with the threshold.
- The correction factor is always ≥ 1, and up to 10⁴ with both factors at the clamp. Classification-time scores are therefore systematically inflated relative to the distributions the threshold was fitted on. Rejection becomes stricter than the F-score optimization intended, by a factor that depends on the input.
- The JavaScript and C# ports behave the same way: training uses plain DTW and classification multiplies by the factor.
- Verdict: real defect, or at least an undocumented inconsistency.
- Correct behavior: score the synthetic positives and negatives exactly as classification does, including correction factors (and lower bounds are then irrelevant).

### 1.7 Synthetic positives are built in the wrong space
- Where:
  - The generator returns a zero vector followed by unit difference vectors (mathematics.h ~l.243–254).
  - Training passes that output directly into feature extraction as if it were a trajectory of positions (jackknife_train.cpp ~l.243–252).
- Feature extraction treats its input as positions. It resamples it by arc length and takes unit differences again. The "positive" samples are therefore paths through direction space, and the vectors compared are turning directions, not the gesture's travel directions.
- The leading zero vector is what an integrating (prefix-sum) reconstruction would use as the origin. That reconstruction step is missing.
- The JavaScript port does the same: a zero vector followed by normalized deltas, and no cumulative sum anywhere in its math module.
- Effect: the positive score distribution does not represent true within-class variation, so the learned threshold is largely arbitrary. The published thresholds were produced with this behavior.
- Verdict: real defect. This matters to the implementer only as a reason not to replicate the threshold procedure (see section 3).

### 1.8 Synthetic negatives
- Where: jackknife_train.cpp ~l.166–227.
- Method:
  - Each negative joins two half-length contiguous windows from randomly chosen templates.
  - The window start is drawn uniformly in [0, len/2).
  - The window length is floor(len/2).
- Hazards:
  - **Short templates:** if a template has fewer than 2 points, len/2 = 0 and the code takes a modulo by zero, which is undefined behavior (in practice a crash).
  - **No templates:** with zero templates, the random template choice also takes a modulo by zero.
  - **Contaminated negatives:** both halves can come from the same template or the same class. A negative is then essentially a positive. Every negative is also scored against every template, including the source classes.
  - **Discontinuity:** the splice point is a jump discontinuity. In IP mode this creates one arbitrary direction vector.
- Correct behavior: require template length ≥ 2 and at least one template, and draw from distinct classes when that is possible.

### 1.9 Histogram range estimation
- Where: jackknife_train.cpp ~l.161–226 and ~l.52–71.
- The maximum score is estimated from the first 51 negatives only (iterations 0–50). Histograms with 1000 bins over [0, max] are then created.
- Negatives from iteration 51 onward, 949 of them, are binned. The first 51 negatives are never binned.
- Out-of-range scores:
  - Any later negative or positive above the estimated maximum is clamped into the last bin.
  - If the maximum is 0, the bin index is score·1000/0, which is inf or NaN.
  - If a score is inf or NaN, converting it to an integer is undefined behavior. The result can be a negative index and an out-of-bounds write.
- Correct behavior:
  - Collect all scores first.
  - Take the range from the finite scores, with a positive floor.
  - Drop non-finite scores.

### 1.10 Threshold selection
- Where: jackknife_train.cpp ~l.101–146.
- Let N(b) and P(b) be the empirical CDFs of negative and positive scores at the upper edge of bin b. Both are normalized to 1 separately. Then:
  - Precision(b) = P(b) / (P(b) + N(b)). This is prevalence-free: it implicitly assumes equal numbers of positives and negatives.
  - Recall(b) = P(b).
  - α = 1/(1+β²), and F(b) = 1 / (α/Precision + (1−α)/Recall). This equals (1+β²)·P·R/(β²P + R), the standard Fβ.
  - The chosen bin is the argmax, taking the first bin on ties because the comparison is strict.
  - The threshold is (index + 0.5) · max ÷ 1000, which is the centre of that bin.
- Edge cases:
  - **Half-bin inconsistency:** the CDF counts everything up to the upper edge of the bin, but the threshold is placed at its centre. Positives in the upper half of the chosen bin are rejected.
  - **Empty leading bins:** where P = N = 0, precision is 0/0 = NaN. F is NaN, and NaN never compares as greater, so those bins are skipped.
  - **Recall zero:** F = 0, never chosen.
  - **No bin wins:** if no bin has F > 0 (possible only through NaNs), the index stays at −1. The threshold becomes −0.5·width, which is negative, so every input is rejected.
  - **No negatives or no positives:** division by zero sum gives NaN everywhere, and then the same −1 outcome. These counts are fixed at 949 and 1000, so this happens only if binning fails.
  - **All positives below all negatives:** F reaches 1 at the first bin where P = 1, so the threshold is tight, at the centre of that bin.
  - **All positives above all negatives:** precision at every bin with positives is ≤ 1/2, and the threshold lands somewhere among the positives (recall pressure pushes it to the top).
- Correct behavior: use the bin's upper edge, or interpolate. Fall back to "no rejection" (+inf) if no valid optimum exists.

### 1.11 Classification mutates shared state, is not reentrant, and ties depend on history
- Where: jackknife_recognizer.h ~l.226–312.
- For each query, the correction factor and lower bound are written into each template object, and the shared template list is sorted in place.
- Consequences:
  - Two concurrent classify calls race.
  - Template order persists between calls. Ties (equal scores) go to whichever template comes first in the current order, because the comparison is strict, so the winner of a tie can depend on earlier queries.
  - If lower bounding is disabled, every lower bound stays at the constructor value −1. The sort is then a no-op on equal keys, and std::sort is not stable.
- Correct behavior: use per-call scratch arrays, a stable order, and a deterministic tie-break (for example, template insertion index).

### 1.12 Pruning logic (correct, but note the semantics)
- A template is skipped if its lower bound exceeds its own rejection threshold, or if the lower bound exceeds the best score so far (strict comparison).
- After full DTW, a template is rejected if its score is strictly greater than its threshold. A score equal to the threshold is accepted.
- The function returns −1 when no template survives.
- Because the templates are sorted by lower bound, the loop could stop at the first lower bound greater than the best score. It does not stop; it uses continue, which is a performance issue only.

### 1.13 DTW band semantics and lengths
- Where: jackknife_recognizer.h ~l.47–105.
- Band:
  - The band is Sakoe–Chiba, counted in indices of the processed sequence: direction vectors in IP mode, resampled points in ED mode.
  - It is inclusive: cell (i, j) is allowed when |i − j| ≤ r, for 1-based i and j with 1 ≤ j ≤ length of the second sequence.
  - The default r = 2 with n = 16 means 15 vectors in IP mode and 16 points in ED mode.
- Recurrence and allocation:
  - The step pattern is symmetric and unweighted. Each cell adds its local cost once, to the minimum of the three predecessors (up, left, diagonal).
  - The full (len₁+1) × (len₂+1) matrix is allocated and set to +inf on every call. Only the banded cells are filled.
- What is returned:
  - The raw accumulated cost at the corner cell, with no path-length normalization.
  - Unequal lengths: if the two lengths differ by more than r, the corner cell is unreachable and the cost is +inf. Lengths are always equal here because both sides are resampled to the same n, but nothing asserts it.
- Other local costs:
  - The ED local cost is the squared Euclidean distance.
  - The IP local cost is 1 − ⟨q_i, t_j⟩, which lies in [0, 2] for unit vectors.

### 1.14 Lower bound validity
- Where:
  - The envelope is built per template over the processed vectors, inclusive window [i−r, i+r] clipped to the valid range, per component minimum and maximum (jackknife_template.h ~l.117–154).
  - The bound itself: jackknife_recognizer.h ~l.111–170.
- ED mode: the bound is the per-component squared distance of the query to the envelope box, summed. This is LB_Keogh.
- IP mode:
  - For each query vector, the code takes the largest dot product achievable with any vector inside the envelope box: the sum over components of q_k·U_k if q_k ≥ 0, else q_k·L_k.
  - That value is clipped to [−1, 1], and the bound adds 1 − (clipped value).
  - This is a valid upper bound on any actual ⟨q_i, t_j⟩ in the band, because each t_j lies in the box. So 1 − bound is a valid lower bound on the local cost.
- Why the bound is valid: every query row appears at least once in any band-constrained warping path, and all costs are non-negative.
- With correction factors: the bound is multiplied by the same factor (> 0) as the full score, so it remains valid. **Correction factors do not invalidate the bound.**
- It becomes invalid only when:
  - vectors are not unit length (1.5);
  - NaNs are present (1.1);
  - query and template lengths differ (the bound indexes the template envelope by query index with no length check, so the read goes out of bounds if the query is longer);
  - the component counts differ (no check anywhere; the vector arithmetic reads out of bounds).

### 1.15 Correction factors
- Where:
  - Feature extraction: jackknife_features.h ~l.101–165.
  - Application: jackknife_recognizer.h ~l.241–257.
- Absolute-distance feature:
  - For each component k, sum over consecutive resampled points of |p_{i,k} − p_{i−1,k}|.
  - The steps are the resampled steps, not raw steps and not unit steps.
  - The vector is then L2-normalized.
- Bounding-box feature:
  - For each component, the maximum minus the minimum over the resampled points. These are positions before z-normalization, not raw points.
  - The vector is then L2-normalized.
- Combined factor: product over the enabled factors of 1 / max(0.01, ⟨f_q, f_t⟩).
  - Both vectors are non-negative and unit length, so each dot product lies in [0, 1]. Each factor lies in [1, 100], and the product in [1, 10⁴].
  - The final score is the combined factor times DTW.
- Hazards:
  - If either feature vector is all zeros (stationary input, or a component that never moves when m = 1), normalization gives NaN. Through the argument-order quirk this becomes the 0.01 clamp, so the factor is 100 (see 1.1).
  - When m = 1, both feature vectors are always [1] (or NaN), so the correction factors do nothing.
- Correct behavior: guard against zero norms, and document that correction factors need m ≥ 2.

### 1.16 One-dimensional data (m = 1)
- In IP mode, unit-normalizing a scalar leaves only its sign (±1). The cost per pair is 0 (same sign) or 2 (opposite sign). The recognizer then compares only the sequence of up/down direction changes, sampled at equal total-variation spacing.
- Zero steps (see 1.1) are more likely in 1-D, at reversals.
- The correction factors are always 1 (see 1.15).
- In ED mode with z-normalization, a constant signal gives NaN (see 1.2).

### 1.17 Queries of a different length than templates
- Raw lengths do not matter, because both sides are resampled to n.
- The dimension must match. If the query has fewer components than the templates, the vector operations index out of bounds. Nothing validates this.

### 1.18 Continuous-mode driver
- Where: session.cpp.in ~l.477–623.
- **Buffer:**
  - A sliding window of filtered points, capped at a fixed number of frames: fps × window seconds, dropping the oldest point.
  - The buffer is cleared on a confirmed detection, and the recent-results history is cleared too.
  - The buffer is not cleared or suspended on tracking loss. Bad frames (any non-finite component) are skipped, so the buffer silently joins across the gap, and the frame counter does not advance for them.
- **After a detection:**
  - The update schedule is not reset, so the recognizer can run again as soon as the buffer holds 2 points.
  - It resamples a 2-point buffer to 16 points. That is a straight line, so all IP direction vectors are identical.
  - The only protection against immediate re-triggering is the agreement rule (below), which needs several more calls.
  - Correct behavior: impose a refractory period, or a minimum buffer duration (or path length) before classifying.
- **Smoothing filter:**
  - It is seeded with the first frame of the session even if that frame is bad. If the first frame contained NaN, the filter state would stay NaN forever. In this dataset, no session's first frame is bad (verified: U101's first bad frame is at index 740), but the code does not guard against it.
  - The smoothing step uses a fixed dt of 1/fps, even though duplicate and bad frames have been removed, so real time intervals vary.
  - Training templates are not smoothed, while the session stream is. The source states the smoothing was added after the paper and that results are "a little higher" than the published ones.
- **Dedup:**
  - At load time, a frame is dropped if its Euclidean distance to the previously kept frame is exactly 0 (meant to remove 60 Hz duplicates of 30 Hz Kinect frames).
  - A NaN distance is not 0, so bad frames are always kept, and only later skipped.
  - For Leap Motion, this dedup also removes genuinely static frames, which compresses time.
- **Agreement rule:**
  - The last K classification results are kept, K being the repeat count. Each new result is compared with all of them.
  - A detection fires only when all K, including the newest, are the same gesture id and that id is not −1.
  - So this is "K identical results in a row" among the recognizer calls, not "twice in a row". Rejections (−1) occupy slots and break agreement.
- **Evaluation bookkeeping:**
  - A command counts as ignored if the next command has the same expected gesture (that is, a retry).
  - A repeated correct detection inside one command counts as a false positive.
  - A wrong detection counts as a false negative for the expected class and a false positive for the detected class. The same command can still later add a true positive, so one command can record both a false negative and a true positive for its class. The scoring is biased.
- **Unused class:** "redrum" (Leap) is excluded from templates and from scoring.

### 1.19 Confusion-matrix aggregation
- Where: confusion_matrices.h ~l.130–141.
- When per-participant matrices are merged, each class's 4-cell matrix is divided by its own sum. A class with no entries for a participant gives 0/0 = NaN, which then poisons the aggregate.
- The aggregate weights all participants equally, regardless of how many commands each had.
- This is evaluation-only code, but it matters if you try to reproduce the published numbers.

### 1.20 Dataset loaders
- **Text loader** (dataset.cpp ~l.284–365):
  - Format:
    - Line 1 is the gesture name.
    - Line 2 is the point count. It is parsed but never used or checked.
    - Line 3 is a separator line of four hash characters.
    - Then come lines of three comma-separated numbers, each line a joint triple. A separator line ends a frame.
    - An empty line, or EOF, ends the file.
  - Each frame's triples are concatenated into one point. In the data, every frame has 21 triples, so m = 63.
  - Field parsing skips exactly one character after each number and assumes three numbers per line, with no validation.
  - A blank line in the middle of the file silently truncates it.
  - A file ending with a trailing separator followed by EOF would add an empty point (m = 0) and break everything downstream. My probe found no such file in the shipped data.
- **Path handling:**
  - The directory loader joins the root path and entry names without a separator, so the root must end with a slash.
  - The function that loads one subject's directory returns null instead of the dataset. If a caller passed null expecting a newly allocated dataset, the allocation is lost. All in-tree callers pass a pre-allocated dataset, so it works by accident.
  - Entries are sorted for determinism.
- **Binary session reader** (session.cpp.in ~l.247–289, ~l.412–470):
  - Each frame is 68 little-endian 4-byte fields:
    - expected gesture id (int)
    - command id (int)
    - three floats (unused: time remaining and on-screen position)
    - 63 floats of position data
  - It type-puns through a union. That is technically undefined behavior in C++ and assumes host endianness.
  - The file size must be an exact multiple of the frame size (asserted only).
  - Session gesture ids are 1-based and map to class names through a fixed table: 14 Kinect classes, then 9 Leap classes.

Dataset facts from my probe (read-only):
- Kinect: 560 training files. Leap: 360 training files.
- Every file has exactly 120 frames and matches its header count.
- Every frame has 63 dimensions.
- Training files contain no exact consecutive duplicate frames, no static samples and no non-finite values.
- Sessions:
  - Kinect sessions contain roughly 40% exact consecutive duplicates (60 Hz logging of 30 Hz data).
  - Leap sessions contain few duplicates.
  - Only U101 (Kinect) has bad frames: 1,649 of them, the first at index 740.

### 1.21 Smaller items
- Random numbers come from the C library's rand():
  - It is not seeded anywhere in the library, so results are deterministic per process but depend on whatever else consumed the generator.
  - The modulo introduces bias.
  - The value is divided by RAND_MAX cast to single-precision float, so the draw is slightly below 1 at the top end.
  - The C# port creates a new clock-seeded generator per call, which can repeat sequences.
- The "uninitialized m-component vector" constructor actually zero-fills, because the underlying resize value-initializes.
- Integer overflow: none is realistic. Counts are small ints, and size_t comparisons are mixed with signed ints throughout.
- Performance only:
  - The O(len²) matrix is allocated per DTW call instead of per band.
  - Erasing from the front of the buffer is O(window) per frame.

---

## 2. Exact behavior where the paper is ambiguous

1. **Order of operations for each sample** (template or query):
   - Arc-length resample the raw points to n points.
   - From the resampled points, compute in one pass:
     - the per-component absolute-step sums;
     - the per-component minimum and maximum;
     - either the n−1 unit direction vectors (IP) or the n points themselves (ED).
   - If enabled, z-normalize the sequence used for matching, per component across the sequence, with divisor n−1.
   - L2-normalize the absolute-distance vector and the bounding-box-width vector.
   - There is no translation or scale normalization other than z-normalization. In IP mode none is needed, because direction vectors are invariant to translation and scale.
2. **Resampling:**
   - Equal arc length, with spacing L/(n−1), using linear interpolation on the raw polyline.
   - The first output point is exactly the first raw point.
   - The last output point is usually the last raw point, appended when floating-point shortfall leaves the walk one point short. Otherwise it is an interpolated point at, or very near, the end.
   - Zero-length raw segments are consumed harmlessly.
3. **Direction vectors:** yes. In IP mode there are exactly n−1 of them, one per consecutive pair of resampled points, each unit-normalized. In ED mode there are n z-normalized points.
4. **Local cost:** IP uses 1 − ⟨q_i, t_j⟩ (not a negative log or an angle). ED uses squared Euclidean distance with no square root.
5. **DTW:** see 1.13. The band is inclusive |i − j| ≤ r over the processed-sequence index. The result is the raw corner cost, unnormalized.
6. **Correction factors:** see 1.15. Both are computed from the resampled points, before z-normalization. The absolute-distance feature uses resampled steps (not unit steps). Both features are L2-normalized; the factor is the reciprocal of their dot product clamped below at 0.01, multiplied across the enabled factors and then by DTW.
7. **Defaults:**
   - Both presets use n = 16, r = 2, lower bounding on, and both correction factors on.
   - The IP preset turns z-normalization off. The ED preset turns it on.
   - The sessions and the user-independent test use the IP preset.
8. **Rejection threshold:**
   - It is not computed from a mean and standard deviation; nothing in the C++ code computes μ or σ.
   - It is chosen per template as the bin centre that maximizes Fβ between CDFs of synthetic positive and negative DTW scores. The scores are raw DTW, without correction factors.
   - Constants:
     - 1000 negatives, of which the first 51 only set the histogram range;
     - 1000 bins;
     - 1000 positives per template;
     - synthetic-variation variance 0.25;
     - β = 1 in sessions;
     - the synthetic resample count and removal count passed by the caller are 6 and 2 (the source says 4 for Kinect and 6 for Leap were used originally; the average is used now).
   - See 1.6–1.10 for the defects.
   - The user-independent test does not train thresholds, so its threshold is +inf and nothing is rejected.
9. **Continuous mode:**

   | Device | Frame rate | Window | Classify every | Window frames | Agreement needed |
   |---|---|---|---|---|---|
   | Kinect | 30 fps | 2.0 s | 5 frames | 60 | 3 identical non-reject results in a row |
   | Leap Motion | 30 fps | 4.0 s | 10 frames | 120 | 4 identical non-reject results in a row |

   - Cleanup: the buffer and the result history are cleared after a detection. There is no other cooldown.
   - Smoothing: a first-order exponential low-pass with a 1 Hz cutoff. The smoothing factor is a = 1/(1 + τ/dt), with τ = 1/(2π·f_c) and dt = 1/30, so a ≈ 0.173. It applies to the stream only, not to the templates.
   - Classification needs a buffer of at least 2 points.
   - The schedule counts only good frames.
10. **User-independent test:** the in-tree "user independent" test trains on one participant's samples and tests on all other participants. This is the reverse of leave-one-out. It runs on Kinect only; Leap is disabled because of mixed handedness.

---

## 3. Patent-sensitive parts: where synthetic data is used

- Synthetic data generation appears in exactly one place: the per-template rejection-threshold training routine (jackknife_train.cpp ~l.152–273). Two generators are used there:
  - stochastic resampling with random point removal (the GPSR technique, implemented in the math helpers, mathematics.h ~l.218–255, built on a random-interval mode of the resampler, ~l.126–142), which produces the positives;
  - splicing two random half-length template windows, which produces the negatives.
- What depends on it: only the rejection threshold stored with each template. Classification itself, meaning resampling, features, DTW, correction factors and lower bounds, does not depend on synthetic data in any way.
- Without that training call:
  - every threshold is +inf, so nothing is rejected (this is the in-tree user-independent configuration);
  - continuous mode then relies only on the K-in-a-row agreement rule, and every window gets a label.
- The resampler's stochastic mode is the only randomness in the core. Uniform resampling (spacing L/(n−1)) is fully deterministic and is all that classification needs.
- Recommendation for a clean implementation:
  - Implement only deterministic uniform arc-length resampling.
  - Take rejection thresholds from a non-synthetic source. Options:
    - user-supplied thresholds;
    - statistics of real held-out positives (for example, leave-one-out DTW scores among a class's own templates);
    - real negatives (scores against other classes' templates, or against recorded non-gesture segments), with the Fβ selection over those real distributions.
  - Given defect 1.7, the reference thresholds are not a quality target worth matching anyway.

---

## 4. Differences from the CHI 2017 paper

I could not retrieve the paper PDF: the UCF URL now redirects to a 404. The comparisons below rely on the repository README plus my recollection of the paper, so verify them against the PDF before relying on them.
- **Smoothing:** the session driver smooths the input stream. The source states the published results did not, and that current results are slightly higher.
- **Synthetic-variation parameters:** the source states the published runs used 4 (Kinect) and 6 (Leap), each with 2 removals. The code now uses 6 for both.
- **Threshold scoring:** the threshold is fitted on raw DTW without correction factors but applied to factor-scaled scores (1.6). The README describes "the threshold which maximizes the F1 score of synthetically generated distributions", without mentioning this mismatch.
- **Synthetic positives:** they are fed to feature extraction as a direction sequence rather than as a reconstructed path (1.7). The cited GPSR method reconstructs points by accumulating the normalized deltas. The code omits that step, although it emits the zero origin the reconstruction would need.
- **Prevalence:** precision is computed from separately normalized CDFs, which implicitly assumes balanced classes. Whether the paper defines precision this way is unconfirmed.
- **"User-independent" test:** it is the single-template-subject variant described in 2.10. Compare it carefully with the paper's protocol before reproducing any numbers.
- **Confusion-matrix aggregation:** the per-class normalization and equal per-participant weighting (1.19) may not match how the paper averaged results.
