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

    /// <summary>
    /// Declares a gesture made of two stable poses: From, then any of To within Within s, held Hold s
    /// (e.g. open palm -> fist; or a snap: thumb on middle finger -> point / palm facing you, Hold 0).
    /// </summary>
    public sealed class PoseSequenceRule
    {
        public string Name, From;
        public string[] To;
        public float Within = 2f, Hold = 1f;
        /// <summary>Identical consecutive raw results that count as having been in From (0 = use the stable pose).
        /// A transitional pose (e.g. opening the hand before a grab) is often too brief to become stable.</summary>
        public int FromRun;

        public bool IsTarget(string pose) => To != null && Array.IndexOf(To, pose) >= 0;
    }

    /// <summary>
    /// Declares a gesture made of a contact that is held, then released into a stable pose: contact Channel on
    /// (ratio &lt; On) for at least MinHold s, then off (ratio &gt; Off), and one of the To poses becomes stable
    /// within Within s of the release (e.g. a finger snap: thumb on middle finger, released into point).
    /// The contact is measured directly, so the held pose does not need its own classifier label.
    /// </summary>
    public sealed class ContactReleaseRule
    {
        public string Name;
        public int Channel;
        public float On = 0.3f, Off = 0.6f, MinHold = 0.3f, Within = 0.5f;
        public string[] To;
        /// <summary>Optional: while the contact is held, Contacts[HoldChannel] must reach HoldPeakAbove at least once
        /// (e.g. the middle finger extended, which tells a snap set-up from a fist or a point). -1 = no condition.</summary>
        public int HoldChannel = -1;
        public float HoldPeakAbove;
        /// <summary>Require an external impulse (e.g. the sound of a snap, see GestureChain.PushImpulse) with peak at
        /// least ImpulseMinPeak and at most ImpulseMaxMs long, within [release - ImpulseBefore, release + ImpulseAfter].
        /// With an impulse required, To may be null (no end pose needed). Thresholds are per chain, i.e. per hand.</summary>
        public bool RequireImpulse;
        public float ImpulseMinPeak = 0.03f, ImpulseMaxMs = 100f, ImpulseBefore = 0.6f, ImpulseAfter = 0.3f;
        /// <summary>Longest time (s) from the last sample still in contact (ratio &lt; On) to the first released sample
        /// (ratio &gt; Off): a snap releases between two consecutive results, a slow hand change does not.</summary>
        public float MaxRelease = float.PositiveInfinity;
    }

    /// <summary>
    /// Declares a drag: while the main contact is on (e.g. a pinch), the Point's in-plane displacement from where the
    /// contact started reaches Distance m within Tolerance degrees of one of Directions. Measured from the contact's
    /// start for as long as it is held, so slow drags count (the stroke primitive needs StrokeConfirm within StrokeWindow).
    /// </summary>
    public sealed class ContactDragRule
    {
        public string Name;
        public Direction Directions = Direction.Right;
        public float Distance = 0.08f, Tolerance = 60f, MinHold = 0.15f, MaxTime = 3f;
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
        public readonly List<ContactReleaseRule> ReleaseRules = new List<ContactReleaseRule>();
        public readonly List<ContactDragRule> DragRules = new List<ContactDragRule>();

        public event Action<GestureEvent> OnGesture;
        /// <summary>The contact primitive switched on or off (time, on, Point at that moment), e.g. a pinch closing/opening.
        /// Raw contact only (not the ContactPose): no pose-stabilizing delay.</summary>
        public event Action<float, bool, float[]> OnContact;
        readonly List<(float t, float peak, float ms)> m_Impulses = new List<(float, float, float)>();

        /// <summary>An external impulse at time t (same clock as samples), e.g. a sharp sound: peak 0..1, length in ms.</summary>
        public void PushImpulse(float t, float peak, float ms)
        {
            m_Impulses.Add((t, peak, ms));
            while (m_Impulses.Count > 0 && t - m_Impulses[0].t > 3f) m_Impulses.RemoveAt(0);
        }

        bool ImpulseNear(ContactReleaseRule r, float released)
        {
            foreach (var (t, peak, ms) in m_Impulses)
                if (t >= released - r.ImpulseBefore && t <= released + r.ImpulseAfter && peak >= r.ImpulseMinPeak && ms <= r.ImpulseMaxMs) return true;
            return false;
        }
        /// <summary>Every finished stroke with the rule result (null when nothing matched); for diagnostics.</summary>
        public event Action<Stroke, string> OnStroke;

        PoseStabilizer m_Pose = new PoseStabilizer();
        ContactDetector m_Contact = new ContactDetector();
        readonly StrokeDetector m_Stroke = new StrokeDetector();
        readonly StrokeDetector m_AnchorStroke = new StrokeDetector();
        readonly Dictionary<PoseSequenceRule, (float lastFrom, bool fired)> m_Seq = new Dictionary<PoseSequenceRule, (float, bool)>();
        // per release rule: contact on, since when, and the time of the last qualifying release (NaN = none)
        readonly Dictionary<ContactReleaseRule, (bool on, float since, float released, float lastIn, float peak)> m_Rel = new Dictionary<ContactReleaseRule, (bool, float, float, float, float)>();
        float[] m_StableAt;
        float m_TrackStart = float.NaN;
        string m_RawPose; int m_RawRun;
        MotionSample? m_ContactStart;
        readonly HashSet<ContactDragRule> m_DragFired = new HashSet<ContactDragRule>();

        public GestureChain(PrimitiveSettings settings) { Settings = settings ?? new PrimitiveSettings(); }

        public string StablePose => m_Pose.Stable;
        public bool Contact => m_Contact.On || (Settings.ContactPose != null && m_Pose.Stable == Settings.ContactPose);

        public void Push(MotionSample s)
        {
            if (float.IsNaN(m_TrackStart)) m_TrackStart = s.Time;
            if (m_Stroke.LastTime > float.NegativeInfinity && s.Time - m_Stroke.LastTime > Settings.Gap)
            {
                EndStroke(m_Stroke.Lost(s.Time, Settings), s.Time, StrokeChannel.Point);
                EndStroke(m_AnchorStroke.Lost(s.Time, Settings), s.Time, StrokeChannel.Anchor);
                ResetState();
                m_TrackStart = s.Time;
            }

            m_RawRun = s.Pose == m_RawPose ? m_RawRun + 1 : 1;
            m_RawPose = s.Pose;
            if (m_Pose.Push(s.Time, s.Pose, Settings.PoseRun))
            {
                m_StableAt = Mid(s.Point, s.Body);
                foreach (var r in PoseRules) if (m_Seq.TryGetValue(r, out var st)) m_Seq[r] = (st.lastFrom, false);
            }
            foreach (var r in PoseRules)
            {
                var (lastFrom, fired) = m_Seq.TryGetValue(r, out var st) ? st : (float.NegativeInfinity, false);
                if (r.FromRun > 0 ? (m_RawPose == r.From && m_RawRun >= r.FromRun) : m_Pose.Stable == r.From) lastFrom = s.Time;
                if (r.IsTarget(m_Pose.Stable) && !fired && m_Pose.StableSince - lastFrom <= r.Within && s.Time - m_Pose.StableSince >= r.Hold)
                {
                    fired = true;
                    OnGesture?.Invoke(new GestureEvent { Time = s.Time, Name = r.Name, Position = m_StableAt });
                }
                m_Seq[r] = (lastFrom, fired);
            }

            foreach (var r in ReleaseRules)
            {
                var (on, since, released, lastIn, peak) = m_Rel.TryGetValue(r, out var st) ? st : (false, 0f, float.NaN, 0f, float.NegativeInfinity);
                float ratio = s.Contacts != null && r.Channel < s.Contacts.Length ? s.Contacts[r.Channel] : float.NaN;
                if (!float.IsNaN(ratio))
                {
                    float hv = r.HoldChannel >= 0 && s.Contacts != null && r.HoldChannel < s.Contacts.Length ? s.Contacts[r.HoldChannel] : float.NaN;
                    if (on && ratio < r.On) { lastIn = s.Time; if (!float.IsNaN(hv)) peak = Math.Max(peak, hv); }
                    if (!on && ratio < r.On) { on = true; since = lastIn = s.Time; peak = float.IsNaN(hv) ? float.NegativeInfinity : hv; }
                    else if (on && ratio > r.Off)
                    {
                        on = false;
                        bool holdOk = r.HoldChannel < 0 || peak >= r.HoldPeakAbove;
                        if (holdOk && lastIn - since >= r.MinHold && s.Time - lastIn <= r.MaxRelease) released = s.Time;
                    }
                }
                if (!float.IsNaN(released))
                {
                    bool fire;
                    float window;
                    if (r.RequireImpulse)
                    {
                        // impulse-confirmed: the sound marks the snap; the end pose is optional
                        fire = ImpulseNear(r, released) && (r.To == null || Array.IndexOf(r.To, m_Pose.Stable) >= 0);
                        window = r.ImpulseAfter + (r.To == null ? 0f : r.Within);
                    }
                    else
                    {
                        // fire when the stable pose is a target within Within s of the release. Not "became stable after it":
                        // the labels during a held contact can flicker too fast to stabilize, leaving the earlier pose stable
                        fire = Array.IndexOf(r.To, m_Pose.Stable) >= 0 && s.Time - released <= r.Within;
                        window = r.Within;
                    }
                    if (fire)
                    {
                        OnGesture?.Invoke(new GestureEvent { Time = s.Time, Name = r.Name, Position = Mid(s.Point, s.Body) });
                        released = float.NaN;
                    }
                    else if (s.Time - released > window) released = float.NaN;
                }
                m_Rel[r] = (on, since, released, lastIn, peak);
            }

            bool wasOn = m_Contact.On;
            m_Contact.Push(s.Contact, Settings.ContactOn, Settings.ContactOff);
            if (m_Contact.On != wasOn) OnContact?.Invoke(s.Time, m_Contact.On, s.Point);
            if (!Contact) { m_ContactStart = null; m_DragFired.Clear(); }
            else
            {
                if (m_ContactStart == null) m_ContactStart = s;
                var a = m_ContactStart.Value;
                float dx = Vec.Dot(Vec.Sub(s.Point, a.Point), a.Right), dy = Vec.Dot(Vec.Sub(s.Point, a.Point), a.Up);
                float dist = (float)Math.Sqrt(dx * dx + dy * dy);
                foreach (var r in DragRules)
                {
                    if (m_DragFired.Contains(r) || dist < r.Distance || s.Time - a.Time < r.MinHold || s.Time - a.Time > r.MaxTime) continue;
                    foreach (var d in new[] { Direction.Right, Direction.Left, Direction.Up, Direction.Down })
                    {
                        if ((r.Directions & d) == 0) continue;
                        float ax = d == Direction.Right ? 1 : d == Direction.Left ? -1 : 0, ay = d == Direction.Up ? 1 : d == Direction.Down ? -1 : 0;
                        double ang = Math.Acos(Math.Max(-1, Math.Min(1, (dx * ax + dy * ay) / dist))) * 180 / Math.PI;
                        if (ang > r.Tolerance) continue;
                        m_DragFired.Add(r);
                        OnGesture?.Invoke(new GestureEvent { Time = s.Time, Name = r.Name.Replace("{dir}", d.ToString().ToLowerInvariant()), Position = (float[])a.Point.Clone() });
                        break;
                    }
                }
            }
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
            if (s.PointTravel >= Settings.MinStroke && !(s.Start - m_TrackStart < Settings.SettleTime))
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
            m_Rel.Clear();
            m_RawPose = null; m_RawRun = 0;
            m_ContactStart = null; m_DragFired.Clear();
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
