# Jackknife reference implementations: clean-room audit findings (C# and JS, with C++ cross-check)

Source examined: the public UCF repository, shallow clone at the current default branch, read only, nothing executed. The CHI 2017 paper text was also read (NSF public-access PDF). This document has prose and math only. Provenance tags look like [CS file:line], [JS file:line], [CPP file:line] and [PAPER p.N]. They exist only so an auditor can check a claim. The implementer does not need them and should not open the cited files.

Notation used throughout: a raw trajectory is a sequence of points x_1..x_L in R^m. The resample count is n and the band half-width is r. Resampled points are p_1..p_n. Raw step vectors are s_i = p_{i+1} - p_i for i = 1..n-1. Unit direction vectors are u_i = s_i / |s_i|.

Licensing note for the clean-room lead: the repository license is an academic, non-commercial, non-sublicensable license (UCF Research Foundation). That alone is a reason for the clean-room separation.

---

## 1. Adversarial QA

Every item below gives the location, whether it is a real defect, and what correct behavior would be.

### 1.1 Zero-length vectors and normalization (division by zero, NaN)

**Unit normalization has no zero guard.** All three ports normalize a vector in place by dividing each component by its Euclidean length, with no epsilon check. A zero vector becomes all-NaN (0/0). [CS Vector.cs:238-248] [JS vector.js:212-219] [CPP vector.h:360-372]

This reaches recognition through four paths:
- **Direction vectors.** In inner-product mode, every step between consecutive resampled points is normalized [CS JackknifeFeatures.cs:117] [JS jackknife_features.js:99]. If two consecutive resampled points coincide, that direction vector is NaN.
  - This happens when the whole input is stationary (path length exactly 0). Resampling then emits n copies of the same point (see 1.2), so every step is zero.
  - It can also happen at the tail. The "append last raw point" fallback can duplicate the last interpolated point to within rounding, which gives a tiny or exactly zero final step.
- **Absolute-distance correction vector.** The component-wise sum of absolute step components is normalized [CS JackknifeFeatures.cs:137] [JS jackknife_features.js:121]. A motionless input makes it NaN.
- **Bounding-box correction vector.** The per-component span of the resampled points is normalized [CS JackknifeFeatures.cs:138] [JS jackknife_features.js:122]. Zero span in every component makes it NaN.
- **Synthetic positive generator.** It normalizes the step between consecutive stochastically resampled points with the same unguarded routine [CS Mathematics.cs:211-218] [JS mathematics.js:184-187].

How NaN propagates:
- In the DTW recurrence, the C# and JS minimum functions return NaN if any argument is NaN, so one NaN cell poisons every reachable downstream cell. The final score becomes NaN.
- In C++, the standard minimum returns whichever argument wins the less-than comparison, so propagation there is order-dependent.
- A NaN score is never rejected, because "NaN greater than threshold" is false. It is also never selected, because "NaN less than best" is false. That template silently drops out.
- A NaN lower bound breaks the template sort:
  - C#: the comparer becomes inconsistent, and the framework sort may throw an "inconsistent comparer" exception or give an arbitrary order.
  - C++: violates strict weak ordering, which is undefined behavior.
  - JS: arbitrary order (see 1.9).
- A NaN correction factor (from a NaN correction vector) gives a NaN lower bound and a NaN score.

Verdict: real defect.

Correct behavior:
- Treat a step whose length is below a small epsilon as the zero vector. Its inner-product local cost is then exactly 1, a neutral cost.
- Or reject or skip degenerate inputs explicitly: if total path length is below epsilon, return "no gesture".
- If a correction vector has zero length, set its factor to the neutral value or to the maximum-penalty clamp. Do not let it become NaN.

### 1.2 Resampling edge cases

The resampler is $1-style arc-length resampling [CS Mathematics.cs:89-182] [JS mathematics.js:82-167] [CPP mathematics.h:109-207].
1. Compute the total polyline length D.
2. Choose n-1 interval fractions. Uniform fractions are 1/(n-1) each. Stochastic fractions are 1 + U·sqrt(12·variance), with U uniform on [0,1), renormalized to sum to 1.
3. Walk the raw polyline. Each time the accumulated distance reaches D times the next fraction, linearly interpolate a new point and make that new point the "previous" point. The raw-point cursor does not advance.

Edge cases found:
- **n = 1.** The fraction 1/(n-1) divides by zero, and the fraction vector has zero length. C# throws at the sum check [CS Mathematics.cs:119-120]. JS only logs a console assertion, which does not throw [JS mathematics.js:105], and later code fails. Correct: require n ≥ 2.
- **Empty input.** Indexing the first point throws (C#), is a TypeError (JS), or is undefined behavior (C++). The feature extractor also reads the component count from the first raw point before resampling [CS JackknifeFeatures.cs:87] [JS jackknife_features.js:62]. Correct: reject empty input.
- **One raw point.** The loop never runs. The output is the single point plus one appended copy, so 2 points, not n.
  - C# throws "count mismatch" [CS Mathematics.cs:177-178].
  - JS asserts silently [JS mathematics.js:166], and the feature loop then reads past the end of the resampled array (undefined → TypeError).
  - C++ asserts only in debug builds; in release it reads out of bounds.
  - Correct: reject inputs with fewer than 2 distinct points.
- **Inputs shorter than n but with ≥ 2 distinct points.** Upsampling by interpolation works correctly. Not a defect.
- **All points identical (D = 0).**
  - The distance to the next point (0) is not strictly less than the remaining distance (0), so the code computes the ratio 0/0 = NaN. A guard clamps NaN or a ratio above 1 to exactly 1 [CS Mathematics.cs:147-153] [JS mathematics.js:130-136].
  - It emits the raw point. Because the cursor does not advance, it repeats this until the output reaches n and returns.
  - Result: n identical points. No infinite loop, but everything downstream is NaN (1.1).
- **Duplicate consecutive raw points** (distance 0 with remaining distance > 0). Handled correctly: the cursor simply advances.
- **Exact hit** (distance equals remaining). Interpolates at ratio 1, which reproduces the raw point. Fine.
- **Early termination.** When the output reaches n, the code returns immediately [CS Mathematics.cs:159-162]. The last output point may then be a hair short of the true endpoint. Benign.
- **Shortfall.** If the walk ends with fewer than n points, exactly one copy of the last raw point is appended [CS Mathematics.cs:174-175]. If the shortfall is 2 or more, C# throws, JS silently returns a short array, and C++ asserts in debug only. Floating-point drift normally yields a shortfall of 0 or 1, so the single append is adequate in practice. Correct: append the true endpoint until the output has n points, or better, force the last point to equal the last raw point.
- **Floating-point accumulation.** The remaining distance is decremented repeatedly, so error accumulates over long raw sequences. This is the source of the ±1 count drift. The total length is computed once and is fine.
- **Interpolation basis.** The interpolated point becomes the new "previous" point, so the next remaining distance is measured from the interpolated point, not from the raw vertex. This is the standard $1 behavior and is correct.
- **Single-precision cast (C++ only).** The stochastic draw divides an integer random value by the maximum random value cast to single precision. The draw is therefore in [0,1] inclusive, not [0,1) [CPP mathematics.h:136-137]. This matters only for stochastic resampling.

### 1.3 Random number generation

- **C#: fresh time-seeded generator per call.** A new generator is constructed on every call of the resampler and of the synthetic generator [CS Mathematics.cs:91, 196]. The project targets .NET Framework 3.5 [CS Jackknife.csproj:12], where the default seed is the millisecond tick count.
  - Calls in the same millisecond produce identical random sequences.
  - The threshold trainer calls the synthetic generator 1000 times per template in a tight loop. Long runs of identical "random" positive samples are therefore expected.
  - Inside one synthetic-generator call, the resampler's generator and the removal generator start from the same seed, so their draws are correlated.
  - Mono, and so Unity, behaves the same way.
  - Verdict: real defect. The positive score histogram collapses onto a few distinct values.
  - Correct: one generator per recognizer, seeded once. Allow an explicit seed for reproducibility.
- **C++: unseeded rand().** The C library generator is never seeded, so runs are deterministic. It also takes `rand() % k` with k = (raw length / 2) [CPP jackknife_train.cpp:182]. A template with fewer than 2 raw points gives modulo by zero, which is undefined behavior (typically SIGFPE).
- **JS: unseeded Math.random.** Results are not reproducible [JS jackknife_recognizer.js:152,157; mathematics.js:99,175].
- **Modulo bias.** Removal indices are drawn as a value in [0,32767) (C#) or [0,65535) (JS) or the C rand range, then reduced modulo the list size. This gives a small modulo bias. Harmless.

### 1.4 Z-score normalization

[CS Mathematics.cs:34-67] [JS mathematics.js:25-61] [CPP mathematics.h:35-83]

Formula: for each component j,
- mean μ_j = (1/N) Σ_i v_ij
- sample variance σ²_j = (1/(N-1)) Σ_i (v_ij − μ_j)²
- σ_j = sqrt(σ²_j)
- v_ij ← (v_ij − μ_j) / σ_j

It is applied to the resampled points in Euclidean mode (N = n), and independently per sequence. This matches the paper's footnote that each sequence is normalized independently.

Hazards:
- **Zero variance in any component.** This happens with a constant coordinate, a padded third dimension for 2-D data, a joint the tracker reports as fixed, or 1-D data with no motion. The division is 0/0 = NaN for every point in that component. Every Euclidean local cost then becomes NaN, and the template or query is unusable. Real defect. Correct: if σ_j < ε, set that component to 0, or divide by max(σ_j, ε).
- **N = 1.** Divides by N−1 = 0. Unreachable with n ≥ 2, but unguarded.
- **Z-normalization with inner product.** If the user enables z-normalization in inner-product mode, the unit direction vectors are z-scored and are no longer unit length. See 1.6 and 1.10.

### 1.5 DTW: band semantics and indexing

[CS JackknifeRecognizer.cs:298-350] [JS jackknife_recognizer.js:229-271] [CPP jackknife_recognizer.h DTW]

**Recurrence.** The cost matrix has dimensions (len1+1) × (len2+1).
- Every cell starts at +∞. The cell (0,0) is set to 0.
- For i = 1..len1, the code visits only j from max(1, i−r) to min(len2, i+r) inclusive. Those cells get
  γ(i,j) = d(a_i, b_j) + min(γ(i−1,j), γ(i,j−1), γ(i−1,j−1)).
- The result is γ(len1, len2), the raw unnormalized path sum.

Observations:
- **Band semantics.** A cell is valid when |i−j| ≤ r, with 1-based indices in both sequences. That is the symmetric Sakoe-Chiba band of half-width r, and it matches the paper. Cells outside the band stay +∞, so a path cannot leave the band. Correct. No off-by-one.
- **Unequal lengths.** If |len1 − len2| > r, the corner cell is unreachable and the result is +∞. In practice every sequence is resampled to the same n, so this never happens. If you ever allow unequal lengths, either use a band scaled to the length ratio or return +∞ explicitly.
- **No path-length normalization.** The score scales with n. Inner-product local cost lies in [0,2] for unit vectors. Squared-Euclidean local cost is unbounded. Thresholds are therefore tied to n and r and must be retrained if either changes.
- **C# shared cost matrix.**
  - The matrix is allocated lazily on the first call, sized from that call's two lengths, and reused afterwards [CS JackknifeRecognizer.cs:301-320]. Each later call re-fills only the current dimensions.
  - A later call with longer sequences, for example after the resample count is changed, indexes out of range.
  - The matrix is shared mutable state, so the recognizer is not reentrant or thread-safe.
  - JS and C++ allocate a fresh matrix per call, which is safe but allocation-heavy.
- **Radius handling.** JS floors the radius in DTW, and in the envelope for the low end only [JS jackknife_template.js:62]. With an integer radius this is equivalent. With a fractional radius the two still agree in practice. C++ stores radius as an 8-bit bitfield (see 1.12).

### 1.6 Inner-product cost and lower bound with non-unit vectors

- **Local cost** is 1 − ⟨a,b⟩ [CS JackknifeRecognizer.cs:338]. The paper says this is in [0,2] "for unit vectors". The code never checks for unit length.
  - Zero vectors, which would occur if 1.1 were guarded with a zero vector, give cost exactly 1.
  - Z-normalized direction vectors, or vectors left non-unit for any other reason, can give ⟨a,b⟩ > 1. The local cost is then negative.
- **Inner-product lower bound** [CS JackknifeRecognizer.cs:244-293] [JS jackknife_recognizer.js:277-315]. For query vector q_i and template envelope U_i, L_i, both taken over template indices i−r..i+r, componentwise:
  - b_i = Σ_j q_ij · (U_ij if q_ij ≥ 0, else L_ij)
  - per-row bound = 1 − clamp(b_i, −1, 1)
  - LB = Σ_i of the per-row bounds.
- **Validity.**
  - With unit vectors, b_i ≥ ⟨q_i, t_k⟩ for every k in the band, because each component product is maximized at the envelope extreme matching its sign. So the per-row bound ≤ min over the band of the local cost.
  - Every warping path visits each query row at least once, inside the same band, and all costs are nonnegative. So LB ≤ DTW. Valid.
  - Clamping b at +1 is harmless, because a true inner product of unit vectors is ≤ 1. Clamping at −1 can never bind for unit vectors.
  - With non-unit vectors (z-normalized direction vectors) the bound is no longer valid. The true local cost can be negative while the bound is clamped to ≥ 0, so LB can exceed DTW and good templates get wrongly culled.
  - Real hazard, reachable in JS through the default constructor (1.10).
- **Euclidean lower bound** (LB_Keogh). Per component, add the squared distance to the envelope when the value lies outside it, else 0. It is valid against the squared-Euclidean DTW with the same band. Correct.
- **Envelope computation** [CS JackknifeTemplate.cs:104-120] [JS jackknife_template.js:57-76]. Componentwise max and min over template indices max(0, i−r) .. min(len−1, i+r), 0-based. It is consistent with the DTW band. Correct.

### 1.7 Correction factors and lower-bound validity

[CS JackknifeRecognizer.cs:185-205] [JS jackknife_recognizer.js:68-90]

Combined factor:
- cf = (1 / max(0.01, ⟨â_q, â_t⟩)) · (1 / max(0.01, ⟨b̂_q, b̂_t⟩)), with each factor applied only if its flag is on.
- â is the unit-normalized absolute-distance vector. b̂ is the unit-normalized bounding-box span vector.
- Both vectors have nonnegative components, so each inner product lies in [0,1] and each factor lies in [1,100].
- The 0.01 floor is not in the paper. It prevents division by zero when the supports are disjoint, for example one gesture moves only in x and the other only in y.
- The maximum combined inflation is 10^4.

Scores:
- Pruning key: cf · LB.
- Full score: cf · DTW.
- Because cf > 0 and the same cf multiplies both, cf·LB ≤ cf·DTW still holds. Lower bounding remains valid once correction factors are applied. Not a defect.

1-D data (m = 1):
- A normalized one-component nonnegative vector is exactly 1, so both factors are identically 1. They carry no information.
- The paper notes the transformation needs dimensionality greater than 1.
- In 1-D, inner-product direction vectors are ±1, so the local cost is only ever 0 or 2. The inner-product measure degenerates to sign matching. Use Euclidean mode for 1-D data.

Zero or NaN correction vectors: see 1.1.

### 1.8 Mismatch between training scores and classification scores

The threshold trainer scores synthetic negatives and positives with plain DTW. It applies no correction factor [CS JackknifeRecognizer.cs:115-117, 153] [JS jackknife_recognizer.js:170-172, 212] [CPP jackknife_train.cpp:201-203, 255-257].

The classifier compares the correction-inflated score cf·DTW, with cf ≥ 1, against the same threshold [CS JackknifeRecognizer.cs:214, 222] [JS jackknife_recognizer.js:102, 118].

The threshold is therefore calibrated on one scale and applied on a larger one. The result is systematic over-rejection, and it is worse for queries whose correction factors are large.

Verdict: real inconsistency in all three ports. The paper is not explicit about whether training scores are inflated; it says only that candidates are compared "using DTW".

Correct: score synthetic samples with exactly the same function used at classification time, including the correction factors. Alternatively, apply the threshold to the uninflated score. Pick one and be consistent.

### 1.9 Template ordering, culling, tie-breaking, mutable shared state

- **Classification mutates the template list.** Every classify call writes each template's cached factor and lower bound, then sorts the template list in place by lower bound [CS JackknifeRecognizer.cs:196-208] [JS jackknife_recognizer.js:83-94].
  - The recognizer is not reentrant. Two concurrent classifications race.
  - In C# the template is a value type held in a list, so it is copied out, modified, and copied back. That is correct but fragile.
- **JS sort comparator is wrong.**
  - It returns a boolean (true when the first lower bound is smaller) [JS jackknife_template.js:79-81]. The array sort expects a signed number, so true is read as +1 (put the first element after the second) and false as 0 (equal).
  - The comparator is inconsistent and never returns a negative value. The resulting order is engine-dependent and roughly descending.
  - Outcome correctness survives because culling is still a valid test. Pruning efficiency is largely lost, and tie-breaking becomes engine-dependent.
  - Real defect, ported from the C++ std::sort convention [CPP jackknife_template.h:163-170].
  - Correct: use a numeric comparison that returns −1, 0 or +1.
- **Tie-breaking.** The best score is replaced only on strictly less, so the first template in sorted order wins a tie. The C# list sort is unstable (introsort), so equal lower bounds give a nondeterministic winner. Minor. Correct: break ties deterministically, for example by insertion index.
- **Culling.**
  - Skip a template when its pruning key exceeds its threshold, or exceeds the best score so far.
  - A score exactly equal to the threshold is accepted, since the rejection test is strictly greater.
  - Because the list is sorted ascending (except in JS), the loop could stop at the first template whose key exceeds the best score. The code continues instead. That is only a missed optimization.
- **Stale lower bound.**
  - When lower bounding is disabled, the cached lower bound keeps its initial −1, so no template is ever culled. That is the intended default.
  - If lower bounding was enabled in an earlier call and then disabled, the stale lower bounds from that call are still used for sorting and culling.
  - Minor real defect. Correct: reset the key every call.
- **Untrained default.** The rejection threshold defaults to +∞, so an untrained recognizer never rejects. The return value −1 means "nothing".

### 1.10 Configuration validation

- **C#** throws in the constructor unless exactly one of {inner product, Euclidean} is set [CS JackknifeRecognizer.cs:52-55].
- **C++** checks the same condition with a debug-only assertion.
- **JS has no such check.**
  - Its configuration constructor defaults to n = 8 and r = 8, with Euclidean, inner product and z-normalize all on [JS jackknife.js:27-79].
  - Both measures on: the feature extractor and DTW silently take the inner-product branch.
  - Z-normalize on: unit direction vectors get z-scored, which breaks unit length and the lower bound (1.6).
  - r = 8 with n = 8 is effectively unconstrained warping.
  - A JS user who forgets to call the preset gets a quietly broken recognizer. Real defect.
- **Undefined function calls (JS).** JS calls an undefined global assert function in unreachable error branches [JS jackknife_recognizer.js:265, 302] and returns an undefined identifier [JS evaluate/session.js:153-154]. These would throw ReferenceError if reached.
- **Correct behavior:** validate that exactly one measure is selected. Warn or forbid z-normalization in inner-product mode. Require n ≥ 2 and 0 ≤ r < n.

### 1.11 Rejection-threshold estimation from score histograms

[CS Distributions.cs:53-135] [JS jackknife_train.js:30-120] [CPP jackknife_train.cpp:32-147]

Procedure:
1. Build two 1000-bin histograms per template over [0, M]. M is the worst negative score seen during the first 51 negative draws (draw indices 0..50 inclusive).
2. Bin index = min(floor(score · 1000 / M), 999). Scores above M pile into the last bin.
3. Convert each histogram to a CDF: normalize by its own total, then take a running sum.
4. At each bin k:
   - recall R_k = CDF_pos(k)
   - "precision" P_k = CDF_pos(k) / (CDF_pos(k) + CDF_neg(k))
   - α = 1/(1+β²)
   - F_k = 1 / (α/P_k + (1−α)/R_k), which equals (1+β²)·P·R / (β²·P + R), the standard F-beta.
5. Pick the first k with the strictly largest F. The threshold is (k + 0.5) · M / 1000, the bin midpoint.

Hazards:
- **M = 0.** If the first 51 negatives all score 0, the bin computation divides by zero.
  - C#: the integer cast of NaN or ∞ gives int.MinValue under unchecked conversion, so the index is negative and throws.
  - JS: the array index is NaN, the increment writes to a property named "NaN", and the sample is silently lost.
  - C++: undefined behavior.
- **NaN scores** (from 1.1) take the same failure paths.
- **The M estimate is fragile.** It comes from only about 51·T early negatives, where T is the template count. Later negatives or positives that score higher are clamped into the top bin. The threshold can never exceed M, and the top bin is overweighted.
- **Weak C# CDF check.** The check that each CDF ends at 1 is written as |last| − 1 ≥ 1e-5 instead of |last − 1| ≥ 1e-5 [CS Distributions.cs:102-103, 108-109]. It catches only overshoot. An all-zero histogram gives a NaN CDF and passes, since NaN comparisons are false. Real defect, but cosmetic. JS uses a non-throwing console assertion; C++ uses a debug-only assertion.
- **Low bins.** Where both CDFs are 0, precision is 0/0 = NaN. F is NaN, and NaN > best is false, so those bins are skipped. Fine.
- **No valid bin.** If no bin produces a positive F, the selected index stays −1. The threshold becomes −0.5·M/1000, which is negative, and every query is rejected. Correct: fall back to +∞, or to a documented percentile.
- **Equal-prior assumption.** "Precision" is computed from CDFs normalized independently. This implicitly assumes equal numbers of positives and negatives. The code uses about 949 negatives and 1000 positives per template, which is close enough. Note it if you change the counts.
- **Ties.** The first (lowest) bin wins ties, which is the conservative choice.
- **Hard-coded β = 1** in the evaluation harness, which is the F1 score.

### 1.12 Language- and platform-specific issues

**C++** (cross-check only):
- The resample count and radius are 8-bit unsigned bitfields [CPP jackknife.h:37, 44]. A value of 256 or more silently wraps.
- Release builds strip every assertion.

**JS:**
- **Copy constructor aliases instead of copying.** The vector "copy" constructor shares the data array [JS vector.js:51-54]. The resampler stores the first raw point and the last raw point by alias [JS mathematics.js:113, 163]. Sample construction stores caller points by reference, without cloning [JS sample.js:40-44]. C# clones [CS Sample.cs:78-82].
  - So in JS, a caller that reuses or mutates point objects after adding a template (a pooled ring buffer, for example) silently corrupts the template's raw trajectory. The template's cached features are mostly safe because they are recomputed into new arrays, but negative splicing and positive synthesis read the raw trajectory later.
  - Real latent defect. Correct: deep-copy input points.
- **Constant fill works only for integers.** The vector constructor fills with a constant only when the constant is an integer or ±∞ [JS vector.js:33-40]. A fractional constant leaves the data undefined. It is latent today because only 0 and ±∞ are used.
- **Assertions do not throw.** Console assertions only log, so failed invariants continue with corrupted state (1.2, 1.11).
- **Falsy defaults mask zero.** The filter cutoff uses "or 1", so a cutoff of 0 becomes 1 Hz [JS filters.js:34]. Sample identifiers default the same way, and the variance argument uses "or 0".

**C#** (no Unity code is present; the project is a .NET 3.5 console app):
- **Allocation per classify.** Each classify allocates a feature object, lists of boxed-in-list doubles (each vector is a list of doubles), a new comparer, and several temporary vectors from operator overloads. In a Unity per-frame loop this creates continual garbage-collector pressure.
- **Unity guidance for a port:**
  - Use pre-allocated flat double arrays.
  - Avoid LINQ and operator-overload temporaries.
  - Do not call per frame; use a cadence (see 3.7).
  - Convert Unity's single-precision vector types to double before accumulating path length and DTW sums.
- **Frame-rate dependence.**
  - The low-pass filter is exponential smoothing with τ = 1/(2π·f_c) and α = 1/(1 + τ/Δt) [CS ExponentialMovingAverage.cs:56-63] [JS filters.js:37-42].
  - It is rate-correct only if Δt is the true elapsed time. The harness passes a constant 1/fps [CS Evaluate/Session.cs:274-276] even though it skips bad frames and removes duplicate frames. The effective cutoff is therefore wrong whenever frames are dropped.
  - Δt = 0 gives α = 0, so the state never updates. A negative Δt gives a nonsensical α.
  - Correct: pass real timestamps and clamp Δt to be greater than 0.
- **Filter seeded with possibly bad frame.** The harness seeds the filter with the first frame even if that frame is flagged bad (NaN or ∞) [CS Evaluate/Session.cs:257] [JS evaluate/session.js:409]. With α < 1, a NaN state never recovers, so the entire session becomes NaN. Real latent defect in the harness. Correct: seed with the first good frame.
- **Cadence counts good frames.** The update cadence counts processed good frames, not wall time.
- **Equality and hash.** Vector equality ignores length mismatch [CS Vector.cs:281-288]. The hash throws on an empty vector [CS Vector.cs:194-202]. Minor.

### 1.13 Continuous-mode buffer handling (evaluation harness)

[CS Evaluate/Session.cs:249-351] [JS evaluate/session.js:401-491]

Loop, per good frame:
1. Filter the frame and append it to a FIFO buffer capped at W = fps · window_seconds frames.
2. Skip if the buffer holds fewer than 2 frames.
3. Skip if the good-frame counter is below the next scheduled update. Otherwise schedule the next update at counter + K.
4. Classify the entire buffer.
5. Push the result into a FIFO of the last C results. If the current result appears C times in that FIFO (all C equal) and is not "none", fire the detection, then clear both the result FIFO and the buffer.

Hazards:
- **No suspension after a detection.**
  - After a detection the buffer restarts empty. The next classification happens K frames later on a buffer of only K frames (about 0.17–0.33 s).
  - Classification fires on nearly motionless micro-buffers, which is the NaN-prone case in 1.1.
  - Repeated short buffers can also re-fire quickly.
  - The paper says the recognizer was suspended for 2 seconds after a detection; the code does not do this (see 2 and 3.7).
- **Linear-time buffer pops.** Removing the head of a list is O(W) per frame. The code comments acknowledge this. Correct: use a ring buffer.
- **"None" counts toward the run.** The C-in-a-row test counts "none" as a value. A run of C "none" results is simply ignored.
- **Duplicate-frame removal at load.** The harness drops a frame whose 63-D point is bit-identical to the previously kept frame. This compensates for a 60 Hz application sampling a 30 Hz sensor [CS Evaluate/Session.cs:185-191]. It is dataset-specific.

### 1.14 Negative-sample splicing details

[CS JackknifeRecognizer.cs:90-108] [JS jackknife_recognizer.js:147-162] [CPP jackknife_train.cpp:166-190]

Procedure: 1000 times, pick 2 templates uniformly with replacement. From each, take a contiguous run of floor(L/2) raw points, where L is that template's raw length, starting at a uniform offset in [0, floor(L/2) − 1]. Concatenate the two runs as raw points, then extract features as for any query.

Hazards:
- **Odd lengths in JS.** JS uses real-valued L/2. For odd L it takes ceil(L/2) points, with the start in [0, (L−1)/2]. The index stays in bounds.
- **L < 2.**
  - C#: takes 0 points. If both picks are like this, the synthetic sequence is empty and feature extraction throws.
  - C++: modulo by zero.
  - JS: takes 1 point.
- **Teleport segment.** The junction between the two halves is a jump in raw space. After arc-length resampling, that single large segment can dominate the negative sample. This differs from the paper (section 2).
- **Same-template splices.** The same template, or the same class, can be drawn twice. The negative distribution then contains half-copies of the positive class, which pulls the threshold downward. This is a questionable choice.

### 1.15 Synthetic-positive output format

All three ports feed the synthetic generator's output into the feature extractor as if it were a trajectory of points. That output is a leading zero vector followed by unit-length step vectors [CS Mathematics.cs:207-220] [JS mathematics.js:180-188] [CPP mathematics.h:243-255].

The paper describes the synthetic sample as a path whose consecutive inter-point distances are normalized to unit length [PAPER p.5854]. That is a cumulative path. Treating the unit steps themselves as positions produces a different shape: points on the unit sphere.

Verdict: the positive distributions, and therefore the learned thresholds, in all three references come from malformed positives. Real defect, shared by all ports.

Implications for the implementer:
- Reference threshold values are not a valid oracle.
- This is moot if the implementer avoids synthetic positives, as section 4 recommends.

---

## 2. Differences between the C#, JS and C++ ports, and against the paper

### 2.1 Port-to-port differences

**Agreement.** All three ports agree on:
- the core recognition math
- inner-product and Euclidean presets: n = 16, r = 2, lower bound on, both correction factors on; z-normalize on only for Euclidean
- the 0.01 floor in the correction factors
- the threshold procedure: 1000 negatives, 1000 positives per template, 1000 bins, histogram range fixed after draw 50, F-beta with bin midpoint
- the synthetic-positive variance of 0.25
- the evaluation parameters (2.2)

**Differences:**

| Area | C# | JS | C++ |
|---|---|---|---|
| Constructor | Throws unless exactly one measure is selected | Defaults to n = 8, r = 8, all flags on, no validation | Debug-only assertion |
| Comparator / sort | Correct three-way comparator | Boolean comparator (broken ordering) | Boolean comparator (correct for its sort API) |
| DTW matrix | Cached and shared (not reentrant, fixed size) | Fresh per call | Fresh per call |
| Random | Time-seeded per call (duplicate-seed defect) | Math.random, unseeded | rand(), unseeded and deterministic |
| Negative splice length (odd L) | floor(L/2) | ceil(L/2) | floor(L/2) |
| Sample storage | Deep-copies points | Stores references (aliasing) | Copies points |
| Failure mode | Exceptions | Silent assertions; undefined helpers | Debug assertions or undefined behavior |
| Parameter width | Unbounded | Unbounded | 8-bit bitfields for n and r |

- **merged.js.** The bundled single-file JS is code-identical to the split JS files. A sorted, comment-stripped line comparison showed no code-line differences.

### 2.2 Code vs. paper

None of the three ports matches the paper on rejection or continuous-mode parameters. On these points the ports agree with each other and disagree with the paper.

1. **Threshold rule.**
   - Paper [PAPER p.5854-5855]:
     - Z-score the negative scores per template, giving mean μ_i and standard deviation σ_i.
     - Z-score the positive scores using the negatives' μ_i and σ_i.
     - Choose a single λ, in standard-deviation units, that minimizes the aggregate count of false positives plus false negatives.
     - The per-template threshold is δ_i = μ_i − λ·σ_i. The automatically chosen λ was about 1.98 (Kinect) and 1.95 (Leap).
   - Code: the 1000-bin CDF and F-beta maximization described in 1.11. There is no z-scoring and no λ.
2. **Negative synthesis.**
   - Paper: pick k training samples. From each, take (n−1)/k consecutive direction vectors (already resampled), then concatenate. There is no positional jump.
   - Code: two half-length raw point runs, spliced and then resampled.
3. **Correction factors.**
   - Paper: plain 1/⟨g(T), g(Q)⟩, with the assumption that components are non-zero.
   - Code: adds the 0.01 floor.
4. **Bounding-box feature.**
   - Paper: the span of the running partial sums of unnormalized steps, taken over partial sums 1..n−1. These are the positions p_{k+1} − p_1 for k ≥ 1, so the start point (the zero partial sum) is excluded.
   - Code: the span over all n resampled points, including p_1.
   - These differ only when p_1 is a strict extreme in some component. Minor.
5. **Absolute-distance feature.**
   - Paper: Σ|p⃗_i| with p⃗_i defined in its Eq. 2 as unit direction vectors. Read literally, this is the componentwise absolute value of unit steps.
   - Code: componentwise absolute value of raw resampled steps s_i.
   - Equal-arc-length resampling makes the |s_i| nearly equal, so after the final normalization the two agree closely. They differ where chords cut corners or the last step is short.
6. **Continuous mode.**
   - Paper [PAPER p.5859]:
     - Classify every 10 frames using the last 4 s of frames.
     - On detection, clear the buffer and suspend for 2 s.
     - Require the same result twice in a row.
   - Code (Kinect): 30 fps, 2 s window, every 5 frames, 3 in a row.
   - Code (Leap): 30 fps, 4 s window, every 10 frames, 4 in a row.
   - Code (both devices): clear on detection, no suspension.
7. **Smoothing.** The code applies 1 Hz exponential smoothing. A comment in the code says the paper's results were produced without smoothing, so the code now scores slightly higher [CS Evaluate/Session.cs:270-273].
8. **Synthetic-positive parameters.** The paper text says the synthetic resampling rate must be hand-tuned. The code calls the trainer with (n = 6, removal count = 2, β = 1). A comment says 4/2 was originally used for Kinect and 6/2 for Leap [CS Evaluate/Session.cs:241-245].
9. **Training score.** The paper says candidates are compared "using DTW". The code also omits correction factors in training, while the paper's figure analysis discusses scores before and after inflation. Either way, the train/classify mismatch in 1.8 is present in all three ports.

**Agreement with the paper:** n = 16, r = 2 (about 10%), direction vectors as unit steps between resampled points, local cost 1 − ⟨·,·⟩, Euclidean fallback with per-sequence z-scoring, both lower bounds, and multiplicative correction-factor combination.

**Bottom line:** on the core recognizer, all three ports match the paper equally. On rejection and continuous mode, none matches the paper.

---

## 3. Behavior the paper leaves ambiguous, as the code resolves it

1. **Resampling.**
   - Arc-length resampling to exactly n points along the raw polyline, by linear interpolation.
   - The first output is the first raw point.
   - Each interpolated point becomes the base for measuring the next interval.
   - Spacing is uniform: D/(n−1).
   - Any shortfall is padded with the last raw point.
   - The output is not normalized for scale or translation (no centering, no scaling). The inner-product measure is scale-invariant by construction. Euclidean mode relies on z-scoring.
2. **Direction vectors.** n−1 vectors u_i = (p_{i+1} − p_i) / |p_{i+1} − p_i|, computed on the resampled points. Euclidean mode instead uses all n resampled points, then z-scores them per component (sample standard deviation, N−1 denominator).
3. **Inner-product local cost.** d = 1 − ⟨u_i, v_j⟩. There is no clamping in the DTW itself. The DTW score is the unnormalized cumulative sum at cell (n−1, n−1), or (n, n) in Euclidean mode.
4. **Correction factors.**
   - Absolute-distance vector: componentwise Σ_i |s_i,j| over raw (not unit) resampled steps, then unit-normalized.
   - Bounding-box vector: componentwise (max − min) over the resampled points, including the first, then unit-normalized. It is not computed from the raw input points.
   - Factor = 1 / max(0.01, ⟨â_q, â_t⟩) × 1 / max(0.01, ⟨b̂_q, b̂_t⟩).
   - In Euclidean mode both vectors are still computed from un-z-scored resampled points.
5. **Order of operations per query.**
   1. Resample.
   2. Compute steps, correction vectors and direction vectors (or points), then z-score if enabled.
   3. For each template: compute cf, then key = cf · LB.
   4. Sort templates by key ascending.
   5. For each template: skip if key > threshold or key > best; otherwise score = cf · DTW; skip if score > threshold; keep the minimum.
   6. Return the class of the minimum, or "none".
6. **Rejection threshold.** Per template, using the histogram, CDF and F-beta procedure in 1.11. The trainer's signature takes the synthetic n, the removal count and β. The synthetic variance is hard-coded at 0.25. The harness uses n = 6, removal count 2 and β = 1. The paper instead uses μ − λσ (2.2).
7. **Continuous-mode constants.**

   | Setting | Code, Kinect | Code, Leap | Paper |
   |---|---|---|---|
   | Window | 2 s (60 frames at 30 fps) | 4 s (120 frames) | 4 s |
   | Cadence | Every 5 good frames | Every 10 good frames | Every 10 frames |
   | Detections required | 3 consecutive identical non-"none" results | 4 consecutive | Same result twice in a row |
   | On detection | Clear buffer and result history; no suspension | Clear buffer and result history; no suspension | Clear buffer, suspend 2 s |
   | Minimum buffer to classify | 2 frames | 2 frames | — |
   | Smoothing | Exponential, 1 Hz cutoff, Δt = 1/30 | Exponential, 1 Hz cutoff, Δt = 1/30 | — |

   The paper explains that the 4 s window is safe because resampling to 16 points makes idle frames contribute little.
8. **Defaults for n and r.**
   - Presets: n = 16, r = 2 for both measures.
   - The configuration comment says r is commonly about 10% of n.
   - The JS bare constructor defaults to n = 8, r = 8. This is not meaningful and is a defect.
   - The user-independent segmented test uses the inner-product preset with no training, so no rejection is applied.
9. **Data assumptions.**
   - Kinect and Leap frames are 63-D: 21 joints × 3.
   - The paper says 1-D or degenerate correction vectors are meaningless. For audio data it used only the absolute-distance factor and disabled z-scoring.
   - The code has no per-factor dimension check.

---

## 4. Patent-sensitive parts: where synthetic generation is used and what depends on it

Synthetic data generation appears only in the per-template rejection-threshold training routine. Nothing in recognition itself depends on it.

**Synthetic positives** (the gesture-path stochastic resampling technique from the authors' UIST 2016 paper):
- Invoked once per template, 1000 times, inside the threshold trainer [CS JackknifeRecognizer.cs:141-157] [JS jackknife_recognizer.js:198-215] [CPP jackknife_train.cpp:232-261].
- Implemented in the math helpers [CS Mathematics.cs:193-221] [JS mathematics.js:169-189] [CPP mathematics.h:218-255].
- It reuses the general resampler's stochastic-interval mode, meaning any variance above 0. Uniform resampling, with variance 0, is the only mode recognition uses.
- An implementer can implement uniform resampling only and never implement random interval draws.

**Synthetic negatives** (splicing):
- Also only in the threshold trainer [CS JackknifeRecognizer.cs:90-136].
- Splicing segments of real samples is a generic data-augmentation idea and is described in the paper itself. It is distinct from the stochastic-resampling technique. Clear its patent status separately if in doubt.

**What depends on synthetic data:**
- Only the per-template rejection thresholds.
- Without training, thresholds stay at +∞. Segmented classification (nearest template, with lower-bound pruning against the best score) works fully. This is exactly how the reference user-independent tests run: training is commented out [JS evaluate/evaluate.js:74-76] [CPP evaluate/user_independent.cpp.in:80].
- Continuous or spotting mode is where thresholds matter. Rejection there needs either real negative examples, a user-supplied threshold, or a non-generative estimate.
- Examples of non-generative estimates:
  - leave-one-out DTW scores among the user's own templates of the same class (positives)
  - scores against other classes' templates, or against real recorded idle motion (negatives)
  - then the paper's μ − λσ rule, or an F-score sweep, over those real scores

**Additional reason to avoid the reference synthetic positives:** as noted in 1.15, all three reference ports feed malformed synthetic positives to the scorer. Their thresholds are therefore not a trustworthy reference to match against.
