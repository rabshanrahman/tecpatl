// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;

namespace Tecpatl
{
    /// <summary>
    /// Static-pose classifier: linear discriminant analysis with Ledoit-Wolf shrinkage over any fixed-length feature
    /// vector. Closed-form fit (class means + one shared covariance), so it can be refitted on device from a user's
    /// own examples. Same algorithm as scikit-learn's StandardScaler + LinearDiscriminantAnalysis(solver="lsqr",
    /// shrinkage="auto"):
    ///   1. standardize features (population std; constant features keep scale 1)
    ///   2. per class: standardize the class's samples, Ledoit-Wolf shrunk covariance, scale back, weight by prior
    ///   3. W = Sigma^-1 means, b = -0.5 mean.W + log prior; predict argmax(W z + b), probability = softmax
    /// Ledoit &amp; Wolf, "A well-conditioned estimator for large-dimensional covariance matrices", J. Multivariate
    /// Analysis 88 (2004).
    /// </summary>
    public sealed class LdaClassifier
    {
        public string[] Classes { get; private set; }
        double[] m_Mean, m_Scale, m_Intercept;
        double[][] m_Coef;
        public int Dim => m_Mean.Length;

        public static LdaClassifier Fit(IReadOnlyList<float[]> X, IReadOnlyList<string> y)
        {
            int n = X.Count, p = X[0].Length;
            var Z = new double[n][];
            for (int i = 0; i < n; i++) { Z[i] = new double[p]; for (int k = 0; k < p; k++) Z[i][k] = X[i][k]; }
            var (mean, scale) = StdScale(Z, Enumerable.Range(0, n).ToArray());
            for (int i = 0; i < n; i++) for (int k = 0; k < p; k++) Z[i][k] = (Z[i][k] - mean[k]) / scale[k];

            var classes = y.Distinct().OrderBy(c => c, StringComparer.Ordinal).ToArray();
            var sigma = new double[p, p];
            var means = new double[classes.Length][];
            var priors = new double[classes.Length];
            for (int c = 0; c < classes.Length; c++)
            {
                var idx = Enumerable.Range(0, n).Where(i => y[i] == classes[c]).ToArray();
                priors[c] = (double)idx.Length / n;
                var (mc, sc) = StdScale(Z, idx);
                means[c] = mc;
                var Xs = idx.Select(i => { var r = new double[p]; for (int k = 0; k < p; k++) r[k] = (Z[i][k] - mc[k]) / sc[k]; return r; }).ToArray();
                var cov = LedoitWolf(Xs, p);
                for (int a = 0; a < p; a++)
                    for (int b = 0; b < p; b++)
                        sigma[a, b] += priors[c] * sc[a] * cov[a, b] * sc[b];
            }

            var L = Cholesky(sigma, p);
            var coef = new double[classes.Length][];
            var intercept = new double[classes.Length];
            for (int c = 0; c < classes.Length; c++)
            {
                coef[c] = CholeskySolve(L, means[c], p);
                double d = 0; for (int k = 0; k < p; k++) d += means[c][k] * coef[c][k];
                intercept[c] = -0.5 * d + Math.Log(priors[c]);
            }
            return new LdaClassifier { Classes = classes, m_Mean = mean, m_Scale = scale, m_Coef = coef, m_Intercept = intercept };
        }

        public string Predict(float[] x, out float probability) => Predict(x, 0, out probability);

        public string Predict(float[] x, int offset, out float probability)
        {
            int p = m_Mean.Length, K = Classes.Length;
            var z = new double[p];
            for (int k = 0; k < p; k++) z[k] = (x[offset + k] - m_Mean[k]) / m_Scale[k];
            double best = double.NegativeInfinity; int arg = 0;
            var s = new double[K];
            for (int c = 0; c < K; c++)
            {
                double v = m_Intercept[c];
                for (int k = 0; k < p; k++) v += m_Coef[c][k] * z[k];
                s[c] = v;
                if (v > best) { best = v; arg = c; }
            }
            double sum = 0; for (int c = 0; c < K; c++) sum += Math.Exp(s[c] - best);
            probability = (float)(1.0 / sum);
            return Classes[arg];
        }

        // ---- persistence: plain text, one value list per line
        public string Save()
        {
            var inv = CultureInfo.InvariantCulture;
            string L(IEnumerable<double> v) => string.Join(",", v.Select(x => x.ToString("R", inv)));
            var sb = new StringBuilder();
            sb.Append("tecpatl-lda 1\n").Append(string.Join(",", Classes)).Append('\n');
            sb.Append(L(m_Mean)).Append('\n').Append(L(m_Scale)).Append('\n').Append(L(m_Intercept)).Append('\n');
            foreach (var row in m_Coef) sb.Append(L(row)).Append('\n');
            return sb.ToString();
        }

        public static LdaClassifier Load(string text)
        {
            var inv = CultureInfo.InvariantCulture;
            var lines = text.Split(new[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);
            if (!lines[0].StartsWith("tecpatl-lda")) throw new FormatException("not a tecpatl LDA model");
            double[] D(string l) => l.Split(',').Select(v => double.Parse(v, inv)).ToArray();
            var m = new LdaClassifier { Classes = lines[1].Split(','), m_Mean = D(lines[2]), m_Scale = D(lines[3]), m_Intercept = D(lines[4]) };
            m.m_Coef = Enumerable.Range(0, m.Classes.Length).Select(c => D(lines[5 + c])).ToArray();
            return m;
        }

        // ---- math
        static (double[] mean, double[] scale) StdScale(double[][] Z, int[] idx)
        {
            int p = Z[0].Length;
            var m = new double[p]; var s = new double[p];
            foreach (int i in idx) for (int k = 0; k < p; k++) m[k] += Z[i][k];
            for (int k = 0; k < p; k++) m[k] /= idx.Length;
            foreach (int i in idx) for (int k = 0; k < p; k++) { double d = Z[i][k] - m[k]; s[k] += d * d; }
            for (int k = 0; k < p; k++) { s[k] = Math.Sqrt(s[k] / idx.Length); if (s[k] == 0) s[k] = 1; }
            return (m, s);
        }

        /// <summary>Ledoit-Wolf shrunk covariance of (re-centered) samples X (n x p).</summary>
        static double[,] LedoitWolf(double[][] X, int p)
        {
            int n = X.Length;
            var mean = new double[p];
            foreach (var r in X) for (int k = 0; k < p; k++) mean[k] += r[k];
            for (int k = 0; k < p; k++) mean[k] /= n;
            var Xc = X.Select(r => { var c = new double[p]; for (int k = 0; k < p; k++) c[k] = r[k] - mean[k]; return c; }).ToArray();

            var xtx = new double[p, p]; var x2tx2 = new double[p, p];
            foreach (var r in Xc)
                for (int a = 0; a < p; a++)
                {
                    double ra = r[a], ra2 = ra * ra;
                    for (int b = 0; b < p; b++) { xtx[a, b] += ra * r[b]; x2tx2[a, b] += ra2 * r[b] * r[b]; }
                }
            var emp = new double[p, p];
            for (int a = 0; a < p; a++) for (int b = 0; b < p; b++) emp[a, b] = xtx[a, b] / n;
            if (n == 1) return emp;

            double trSum = 0; for (int k = 0; k < p; k++) trSum += emp[k, k];
            double mu = trSum / p, beta_ = 0, delta_ = 0;
            for (int a = 0; a < p; a++) for (int b = 0; b < p; b++) { beta_ += x2tx2[a, b]; delta_ += xtx[a, b] * xtx[a, b]; }
            delta_ /= (double)n * n;
            double beta = 1.0 / ((double)p * n) * (beta_ / n - delta_);
            double delta = (delta_ - 2.0 * mu * trSum + p * mu * mu) / p;
            beta = Math.Min(beta, delta);
            double sh = beta == 0 ? 0 : beta / delta;
            for (int a = 0; a < p; a++) for (int b = 0; b < p; b++) emp[a, b] *= 1 - sh;
            for (int k = 0; k < p; k++) emp[k, k] += sh * mu;
            return emp;
        }

        static double[,] Cholesky(double[,] A, int p)
        {
            var L = new double[p, p];
            for (int i = 0; i < p; i++)
                for (int j = 0; j <= i; j++)
                {
                    double s = A[i, j];
                    for (int k = 0; k < j; k++) s -= L[i, k] * L[j, k];
                    if (i == j)
                    {
                        if (s <= 0) throw new InvalidOperationException("covariance is not positive definite");
                        L[i, i] = Math.Sqrt(s);
                    }
                    else L[i, j] = s / L[j, j];
                }
            return L;
        }

        static double[] CholeskySolve(double[,] L, double[] b, int p)
        {
            var y = new double[p];
            for (int i = 0; i < p; i++) { double s = b[i]; for (int k = 0; k < i; k++) s -= L[i, k] * y[k]; y[i] = s / L[i, i]; }
            var x = new double[p];
            for (int i = p - 1; i >= 0; i--) { double s = y[i]; for (int k = i + 1; k < p; k++) s -= L[k, i] * x[k]; x[i] = s / L[i, i]; }
            return x;
        }
    }
}
