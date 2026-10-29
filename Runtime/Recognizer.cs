// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;

namespace Tecpatl
{
    public struct Result
    {
        public string Label;      // winning class, or null if no templates
        public float Score;       // corrected DTW score of the winning template (lower = more similar)
        public bool Accepted;     // false when rejected by the winning template's threshold
        public int TemplateIndex;
    }

    /// <summary>A stored example: resampled, with direction vectors, correction features and envelopes.</summary>
    public sealed class Template
    {
        public string Label;
        public float[][] Directions;   // unit steps (SPEC §3)
        public float[][] ZScored;      // for the Euclidean fallback
        public float[] Abs;            // Eq. 10, unit length
        public float[] Box;            // Eq. 11, unit length
        public float[][] Upper, Lower; // Eq. 6 envelopes over Directions
        public float Threshold = float.PositiveInfinity;   // rejection: accept when Score <= Threshold

        public static Template Build(string label, IReadOnlyList<float[]> frames, int n, int band)
        {
            var pts = GesturePath.Resample(frames, n);
            var steps = GesturePath.Steps(pts);
            var t = new Template
            {
                Label = label,
                Directions = GesturePath.Directions(steps),
                ZScored = GesturePath.ZScore(pts),
                Abs = Correction.AbsDistance(steps),
                Box = Correction.BoundingBox(steps),
            };
            Dtw.Envelopes(t.Directions, band, out t.Upper, out t.Lower);
            return t;
        }
    }

    /// <summary>Correction factor features (SPEC §6).</summary>
    public static class Correction
    {
        /// <summary>Component-wise sum of |step| over the path, unit length (Eq. 10).</summary>
        public static float[] AbsDistance(float[][] steps)
        {
            var g = new float[steps[0].Length];
            foreach (var s in steps) for (int k = 0; k < g.Length; k++) g[k] += Math.Abs(s[k]);
            return Vec.Normalized(g);
        }

        /// <summary>Per-component extent of the cumulative path, unit length (Eq. 11).</summary>
        public static float[] BoundingBox(float[][] steps)
        {
            int m = steps[0].Length;
            var cum = new float[m]; var max = new float[m]; var min = new float[m];
            foreach (var s in steps)
                for (int k = 0; k < m; k++)
                {
                    cum[k] += s[k];
                    max[k] = Math.Max(max[k], cum[k]);
                    min[k] = Math.Min(min[k], cum[k]);
                }
            var g = new float[m];
            for (int k = 0; k < m; k++) g[k] = max[k] - min[k];
            return Vec.Normalized(g);
        }

        /// <summary>f = 1 / ⟨g(T), g(Q)⟩ clamped to [1, ∞) (Eq. 9).</summary>
        public static float Factor(float[] gt, float[] gq)
        {
            float d = Vec.Dot(gt, gq);
            return d <= 1e-6f ? 1e6f : Math.Max(1f, 1f / d);
        }
    }

    /// <summary>1-NN DTW recognizer over stored templates (SPEC §7–9).</summary>
    public sealed class Recognizer
    {
        public readonly int Resample, Band;
        public LocalCost Cost = LocalCost.InnerProduct;
        public bool UseCorrectionFactors = true;
        public bool UseLowerBound = true;
        public readonly List<Template> Templates = new List<Template>();

        public Recognizer(int resample = 16, int band = 2) { Resample = resample; Band = band; }

        public Template AddTemplate(string label, IReadOnlyList<float[]> frames)
        {
            var t = Template.Build(label, frames, Resample, Band);
            Templates.Add(t);
            return t;
        }

        /// <summary>Corrected score of a prepared query against one template.</summary>
        public float Score(Template t, Query q)
        {
            float s = Cost == LocalCost.InnerProduct
                ? Dtw.Score(t.Directions, q.Directions, Band, LocalCost.InnerProduct)
                : Dtw.Score(t.ZScored, q.ZScored, Band, LocalCost.SquaredEuclidean);
            if (UseCorrectionFactors) s *= Correction.Factor(t.Abs, q.Abs) * Correction.Factor(t.Box, q.Box);
            return s;
        }

        public Result Classify(IReadOnlyList<float[]> frames) => Classify(Prepare(frames));

        public Result Classify(Query q)
        {
            var best = new Result { Label = null, Score = float.PositiveInfinity, TemplateIndex = -1 };
            for (int i = 0; i < Templates.Count; i++)
            {
                var t = Templates[i];
                if (UseLowerBound && Cost == LocalCost.InnerProduct && Dtw.LowerBoundIP(t.Upper, t.Lower, q.Directions) >= best.Score)
                    continue;   // correction factors only inflate, so the bound on raw DTW is a valid prune
                float s = Score(t, q);
                if (s < best.Score) { best.Score = s; best.Label = t.Label; best.TemplateIndex = i; }
            }
            best.Accepted = best.TemplateIndex >= 0 && best.Score <= Templates[best.TemplateIndex].Threshold;
            return best;
        }

        public Query Prepare(IReadOnlyList<float[]> frames)
        {
            var pts = GesturePath.Resample(frames, Resample);
            var steps = GesturePath.Steps(pts);
            return new Query
            {
                Directions = GesturePath.Directions(steps),
                ZScored = GesturePath.ZScore(pts),
                Abs = Correction.AbsDistance(steps),
                Box = Correction.BoundingBox(steps),
            };
        }

        /// <summary>
        /// Rejection thresholds from REAL data (SPEC §9): for each template, scores of real positives of its
        /// class (not stored as templates) and of real negative windows; the threshold maximizing F1 is kept.
        /// Templates with no positives get the negatives' minimum score (reject everything seen as negative).
        /// </summary>
        /// <param name="beta">F-beta weighting: beta &lt; 1 favours precision (fewer false activations), &gt; 1 favours recall.</param>
        public void Calibrate(IEnumerable<(string label, IReadOnlyList<float[]> frames)> positives,
                              IEnumerable<IReadOnlyList<float[]>> negatives, float beta = 1f)
        {
            float b2 = beta * beta;
            var pos = new List<(string, Query)>();
            foreach (var (l, f) in positives) pos.Add((l, Prepare(f)));
            var neg = new List<Query>();
            foreach (var f in negatives) neg.Add(Prepare(f));

            foreach (var t in Templates)
            {
                var ps = new List<float>(); var ns = new List<float>();
                foreach (var (l, q) in pos) if (l == t.Label) ps.Add(Score(t, q));
                foreach (var q in neg) ns.Add(Score(t, q));
                if (ns.Count == 0) { t.Threshold = float.PositiveInfinity; continue; }
                if (ps.Count == 0) { ns.Sort(); t.Threshold = ns[0] * 0.999f; continue; }
                var cands = new List<float>(ps); cands.AddRange(ns); cands.Sort();
                float bestF1 = -1f, bestThr = cands[0];
                foreach (float thr in cands)
                {
                    int tp = 0, fp = 0, fn = 0;
                    foreach (float s in ps) if (s <= thr) tp++; else fn++;
                    foreach (float s in ns) if (s <= thr) fp++;
                    float f1 = tp == 0 ? 0f : (1f + b2) * tp / ((1f + b2) * tp + fp + b2 * fn);
                    if (f1 > bestF1) { bestF1 = f1; bestThr = thr; }
                }
                t.Threshold = bestThr;
            }
        }
    }

    public sealed class Query
    {
        public float[][] Directions, ZScored;
        public float[] Abs, Box;
    }
}
