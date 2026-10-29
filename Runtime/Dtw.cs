// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;

namespace Tecpatl
{
    public enum LocalCost { InnerProduct, SquaredEuclidean }

    /// <summary>Dynamic time warping with a Sakoe–Chiba band (SPEC §4–5).</summary>
    public static class Dtw
    {
        /// <summary>Inner-product dissimilarity 1 - ⟨t, q⟩ for unit vectors (Eq. 5).</summary>
        public static float CostIP(float[] t, float[] q) => 1f - Vec.Dot(t, q);

        public static float CostED(float[] t, float[] q)
        {
            float s = 0f;
            for (int i = 0; i < t.Length; i++) { float d = t[i] - q[i]; s += d * d; }
            return s;
        }

        /// <summary>DTW score between two equal-length sequences within band r (Eq. 4).</summary>
        public static float Score(float[][] t, float[][] q, int r, LocalCost cost)
        {
            int n = t.Length, m = q.Length;
            var g = new float[n + 1, m + 1];
            for (int i = 0; i <= n; i++) for (int j = 0; j <= m; j++) g[i, j] = float.PositiveInfinity;
            g[0, 0] = 0f;
            for (int i = 1; i <= n; i++)
            {
                int lo = Math.Max(1, i - r), hi = Math.Min(m, i + r);
                for (int j = lo; j <= hi; j++)
                {
                    float d = cost == LocalCost.InnerProduct ? CostIP(t[i - 1], q[j - 1]) : CostED(t[i - 1], q[j - 1]);
                    g[i, j] = d + Math.Min(g[i - 1, j - 1], Math.Min(g[i - 1, j], g[i, j - 1]));
                }
            }
            return g[n, m];
        }

        /// <summary>Per-component upper/lower envelopes of a template over the band (Eq. 6).</summary>
        public static void Envelopes(float[][] t, int r, out float[][] upper, out float[][] lower)
        {
            int n = t.Length, m = t[0].Length;
            upper = new float[n][]; lower = new float[n][];
            for (int i = 0; i < n; i++)
            {
                upper[i] = new float[m]; lower[i] = new float[m];
                for (int k = 0; k < m; k++) { upper[i][k] = float.NegativeInfinity; lower[i][k] = float.PositiveInfinity; }
                for (int j = Math.Max(0, i - r); j <= Math.Min(n - 1, i + r); j++)
                    for (int k = 0; k < m; k++)
                    {
                        upper[i][k] = Math.Max(upper[i][k], t[j][k]);
                        lower[i][k] = Math.Min(lower[i][k], t[j][k]);
                    }
            }
        }

        /// <summary>Lower bound of the inner-product DTW score (Eq. 8).</summary>
        public static float LowerBoundIP(float[][] upper, float[][] lower, float[][] q)
        {
            float s = 0f;
            for (int i = 0; i < q.Length; i++)
            {
                float dot = 0f;
                for (int k = 0; k < q[i].Length; k++) dot += (q[i][k] >= 0f ? upper[i][k] : lower[i][k]) * q[i][k];
                s += 1f - Math.Min(1f, Math.Max(-1f, dot));
            }
            return s;
        }
    }
}
