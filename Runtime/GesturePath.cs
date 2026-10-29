// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;

namespace Tecpatl
{
    /// <summary>Gesture path preprocessing (GesturePath): resampling (SPEC §2) and direction vectors (SPEC §3).</summary>
    public static class GesturePath
    {
        /// <summary>Resamples a point sequence to n points equidistant along its arc length.</summary>
        public static float[][] Resample(IReadOnlyList<float[]> points, int n)
        {
            if (points.Count == 0) throw new ArgumentException("empty sequence");
            int m = points[0].Length;
            if (points.Count == 1 || n < 2)
            {
                var same = new float[n][];
                for (int i = 0; i < n; i++) same[i] = (float[])points[0].Clone();
                return same;
            }
            float total = 0f;
            for (int i = 1; i < points.Count; i++) total += Vec.Dist(points[i - 1], points[i]);
            float step = total / (n - 1);
            var outPts = new List<float[]>(n) { (float[])points[0].Clone() };
            if (step <= 0f)
            {
                while (outPts.Count < n) outPts.Add((float[])points[0].Clone());
                return outPts.ToArray();
            }
            float acc = 0f;
            var prev = points[0];
            for (int i = 1; i < points.Count && outPts.Count < n; i++)
            {
                var cur = points[i];
                float seg = Vec.Dist(prev, cur);
                while (acc + seg >= step && outPts.Count < n)
                {
                    float t = (step - acc) / seg;
                    var p = new float[m];
                    for (int k = 0; k < m; k++) p[k] = prev[k] + t * (cur[k] - prev[k]);
                    outPts.Add(p);
                    prev = p;
                    seg = Vec.Dist(prev, cur);
                    acc = 0f;
                }
                acc += seg;
                prev = cur;
            }
            while (outPts.Count < n) outPts.Add((float[])points[points.Count - 1].Clone());
            return outPts.ToArray();
        }

        /// <summary>Unnormalized steps p[i+1] - p[i] (n-1 vectors).</summary>
        public static float[][] Steps(float[][] resampled)
        {
            var r = new float[resampled.Length - 1][];
            for (int i = 0; i < r.Length; i++) r[i] = Vec.Sub(resampled[i + 1], resampled[i]);
            return r;
        }

        /// <summary>Gesture path direction vectors: unit-length steps (SPEC §3, Eq. 2).</summary>
        public static float[][] Directions(float[][] steps)
        {
            var r = new float[steps.Length][];
            for (int i = 0; i < steps.Length; i++) r[i] = Vec.Normalized(steps[i]);
            return r;
        }

        /// <summary>Per-sequence z-score normalization (for the Euclidean fallback cost).</summary>
        public static float[][] ZScore(float[][] seq)
        {
            int n = seq.Length, m = seq[0].Length;
            var mean = new float[m]; var sd = new float[m];
            foreach (var p in seq) for (int k = 0; k < m; k++) mean[k] += p[k] / n;
            foreach (var p in seq) for (int k = 0; k < m; k++) { float d = p[k] - mean[k]; sd[k] += d * d / n; }
            var r = new float[n][];
            for (int i = 0; i < n; i++)
            {
                r[i] = new float[m];
                for (int k = 0; k < m; k++) r[i][k] = (seq[i][k] - mean[k]) / (float)Math.Max(Math.Sqrt(sd[k]), 1e-6);
            }
            return r;
        }
    }
}
