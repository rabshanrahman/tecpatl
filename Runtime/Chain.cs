// Copyright 2026 Deep Auras. Licensed under the Apache License, Version 2.0.
using System;
using System.Collections.Generic;

namespace Tecpatl
{
    public enum Require { Any, Yes, No }

    /// <summary>
    /// Declares a gesture made of one stroke plus conditions on the other primitives at its start/end.
    /// Rules are tried in order when a stroke ends; the first match wins. A rule with a null Name swallows the
    /// stroke (no event), so an earlier, narrower rule can claim a class of strokes.
    /// </summary>
    public sealed class StrokeRule
    {
        /// <summary>Event name; "{dir}" is replaced with left/right/up/down. Null = consume without an event.</summary>
        public string Name;
        /// <summary>Stroke channel the rule applies to: the Point (e.g. fingertip) or the Anchor (e.g. knuckles).</summary>
        public StrokeChannel Channel = StrokeChannel.Point;
        public Direction Directions = Direction.Any;
        /// <summary>Stable pose required at stroke onset; null = any.</summary>
        public string PoseAtStart;
        public Require ContactAtStart = Require.Any, ContactAtEnd = Require.Any;
        public float MinTurnDeg = 0f, MaxTurnDeg = float.PositiveInfinity;
        /// <summary>Body travel / Point travel: high = the whole limb moved, low = only the Point (a finger) moved. Min inclusive, Max exclusive.</summary>
        public float MinBodyRatio = 0f, MaxBodyRatio = float.PositiveInfinity;

        public bool Matches(Stroke s)
        {
            if (s.Channel != Channel || (Directions & s.Direction) == 0) return false;
            if (PoseAtStart != null && s.PoseAtStart != PoseAtStart) return false;
            if (!Ok(ContactAtStart, s.ContactAtStart) || !Ok(ContactAtEnd, s.ContactAtEnd)) return false;
            if (s.TurnDeg < MinTurnDeg || s.TurnDeg >= MaxTurnDeg) return false;
            float ratio = s.PointTravel > 1e-9f ? s.BodyTravel / s.PointTravel : float.PositiveInfinity;
            return ratio >= MinBodyRatio && ratio < MaxBodyRatio;
        }

        public string EventName(Stroke s) => Name?.Replace("{dir}", s.Direction.ToString().ToLowerInvariant());

        static bool Ok(Require r, bool v) => r == Require.Any || (r == Require.Yes) == v;
    }

    /// <summary>Declares a gesture made of two stable poses: From, then To within Within s, To held Hold s.</summary>
    public sealed class PoseSequenceRule
    {
        public string Name, From, To;
        public float Within = 2f, Hold = 1f;
    }

    public struct GestureEvent
    {
        public float Time;
        public string Name;
        /// <summary>Where it happened (world frame): strokes = Point at motion onset; pose sequences = midpoint of Point and Body when To became stable.</summary>
        public float[] Position;
    }

    /// <summary>
    /// Primitives + chaining for one limb (one instance per hand). Feed one MotionSample per tracker result and call
    /// Tick once per result (any limb) so strokes end when the limb leaves tracking.
    /// </summary>
    public sealed class GestureChain
    {
        public readonly PrimitiveSettings Settings;
        public readonly List<StrokeRule> StrokeRules = new List<StrokeRule>();
        public readonly List<PoseSequenceRule> PoseRules = new List<PoseSequenceRule>();

        public event Action<GestureEvent> OnGesture;
        /// <summary>Every finished stroke with the rule result (null when nothing matched); for diagnostics.</summary>
        public event Action<Stroke, string> OnStroke;

        PoseStabilizer m_Pose = new PoseStabilizer();
        ContactDetector m_Contact = new ContactDetector();
        readonly StrokeDetector m_Stroke = new StrokeDetector();
        readonly StrokeDetector m_AnchorStroke = new StrokeDetector();
        readonly Dictionary<PoseSequenceRule, (float lastFrom, bool fired)> m_Seq = new Dictionary<PoseSequenceRule, (float, bool)>();
        float[] m_StableAt;

        public GestureChain(PrimitiveSettings settings) { Settings = settings ?? new PrimitiveSettings(); }

        public string StablePose => m_Pose.Stable;
        public bool Contact => m_Contact.On || (Settings.ContactPose != null && m_Pose.Stable == Settings.ContactPose);

        public void Push(MotionSample s)
        {
            if (m_Stroke.LastTime > float.NegativeInfinity && s.Time - m_Stroke.LastTime > Settings.Gap)
            {
                EndStroke(m_Stroke.Lost(s.Time, Settings), s.Time, StrokeChannel.Point);
                EndStroke(m_AnchorStroke.Lost(s.Time, Settings), s.Time, StrokeChannel.Anchor);
                ResetState();
            }

            if (m_Pose.Push(s.Time, s.Pose, Settings.PoseRun))
            {
                m_StableAt = Mid(s.Point, s.Body);
                foreach (var r in PoseRules) if (m_Seq.TryGetValue(r, out var st)) m_Seq[r] = (st.lastFrom, false);
            }
            foreach (var r in PoseRules)
            {
                var (lastFrom, fired) = m_Seq.TryGetValue(r, out var st) ? st : (float.NegativeInfinity, false);
                if (m_Pose.Stable == r.From) lastFrom = s.Time;
                if (m_Pose.Stable == r.To && !fired && m_Pose.StableSince - lastFrom <= r.Within && s.Time - m_Pose.StableSince >= r.Hold)
                {
                    fired = true;
                    OnGesture?.Invoke(new GestureEvent { Time = s.Time, Name = r.Name, Position = m_StableAt });
                }
                m_Seq[r] = (lastFrom, fired);
            }

            m_Contact.Push(s.Contact, Settings.ContactOn, Settings.ContactOff);
            EndStroke(m_Stroke.Push(s, m_Pose.Stable, Contact, Settings), s.Time, StrokeChannel.Point);
            if (s.Anchor != null)
            {
                var a = s; a.Point = s.Anchor;   // the anchor channel runs the same stroke primitive on the Anchor
                EndStroke(m_AnchorStroke.Push(a, m_Pose.Stable, Contact, Settings), s.Time, StrokeChannel.Anchor);
            }
        }

        /// <summary>Call once per tracker result: ends the stroke of a limb that left tracking.</summary>
        public void Tick(float now)
        {
            if (m_Stroke.LastTime > float.NegativeInfinity && now - m_Stroke.LastTime > Settings.Gap)
                EndStroke(m_Stroke.Lost(now, Settings), now, StrokeChannel.Point);
            if (m_AnchorStroke.LastTime > float.NegativeInfinity && now - m_AnchorStroke.LastTime > Settings.Gap)
                EndStroke(m_AnchorStroke.Lost(now, Settings), now, StrokeChannel.Anchor);
        }

        void EndStroke(Stroke? stroke, float now, StrokeChannel channel)
        {
            if (stroke == null) return;
            var s = stroke.Value;
            s.Channel = channel;
            string name = null;
            if (s.PointTravel >= Settings.MinStroke)
                foreach (var r in StrokeRules)
                    if (r.Matches(s)) { name = r.EventName(s); break; }
            OnStroke?.Invoke(s, name);
            if (name != null) OnGesture?.Invoke(new GestureEvent { Time = now, Name = name, Position = s.Origin });
        }

        void ResetState()
        {
            m_Pose = new PoseStabilizer();
            m_Contact = new ContactDetector();
            m_Stroke.Reset();
            m_AnchorStroke.Reset();
            m_Seq.Clear();
        }

        static float[] Mid(float[] a, float[] b)
        {
            if (a == null) return b;
            if (b == null) return a;
            var r = new float[a.Length];
            for (int i = 0; i < a.Length; i++) r[i] = 0.5f * (a[i] + b[i]);
            return r;
        }
    }
}
