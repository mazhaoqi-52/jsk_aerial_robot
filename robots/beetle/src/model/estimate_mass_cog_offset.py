#!/usr/bin/env python3
"""
Estimate mass error and CoG offset from the hover external-wrench estimate.

Theory (static level hover; formation wrench about and expressed in
assembly_cog, whose axes are approximately aligned with world at level hover):
  Unmodeled gravity (true mass m_true = m_model + dm at true CoG = model CoG + r)
  appears in the momentum-observer estimate as a constant external wrench:

    f_z   = -dm * g              ->  dm  = -f_z / g
    tau_x = -r_y * m_true * g    ->  r_y = -tau_x / (m_true * g)
    tau_y = +r_x * m_true * g    ->  r_x = +tau_y / (m_true * g)

  Gravity is vertical, so it can NOT produce f_x, f_y or tau_z. Whatever
  remains in those channels is a NON-gravity model error (gimbal zero offset,
  rotor thrust asymmetry, thrust-curve mismatch, ...). r_z is unobservable at
  hover (needs a lateral-force segment).

Usage:
  rosrun beetle estimate_mass_cog_offset.py BAG --mass 8.497 \
      [--topic /assemble/formation_observer/est_ext_wrench] \
      [--start 30] [--end 60]

  --start/--end are seconds relative to bag start; pick a quiet hover window.
  Also works with a per-module observer topic (geometry_msgs/WrenchStamped,
  module CoG frame) if --mass is that module's model mass.
"""

import argparse
import math
import sys

import rosbag

G = 9.797  # same as aerial_robot_estimation::G


def mean_std(values):
    n = len(values)
    m = sum(values) / n
    var = sum((v - m) ** 2 for v in values) / n
    return m, math.sqrt(var)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag file")
    parser.add_argument("--topic", default="/assemble/formation_observer/est_ext_wrench",
                        help="WrenchStamped topic (default: formation observer in assembly_cog)")
    parser.add_argument("--mass", type=float, required=True,
                        help="model mass [kg] matching the topic (formation or module)")
    parser.add_argument("--start", type=float, default=None,
                        help="window start [s], relative to bag start")
    parser.add_argument("--end", type=float, default=None,
                        help="window end [s], relative to bag start")
    args = parser.parse_args()

    samples = [[] for _ in range(6)]
    t_first = t_last = None
    with rosbag.Bag(args.bag) as bag:
        bag_start = bag.get_start_time()
        for _, msg, t in bag.read_messages(topics=[args.topic]):
            t_rel = t.to_sec() - bag_start
            if args.start is not None and t_rel < args.start:
                continue
            if args.end is not None and t_rel > args.end:
                break
            w = msg.wrench.wrench if hasattr(msg.wrench, "wrench") else msg.wrench
            for i, v in enumerate((w.force.x, w.force.y, w.force.z,
                                   w.torque.x, w.torque.y, w.torque.z)):
                samples[i].append(v)
            if t_first is None:
                t_first = t_rel
            t_last = t_rel

    if not samples[0]:
        sys.exit("no messages on %s in the given window" % args.topic)

    stats = [mean_std(s) for s in samples]
    labels = ["f_x [N]", "f_y [N]", "f_z [N]",
              "tau_x [Nm]", "tau_y [Nm]", "tau_z [Nm]"]

    print("bag     : %s" % args.bag)
    print("topic   : %s" % args.topic)
    print("window  : %.1f - %.1f s (rel), %d samples" %
          (t_first, t_last, len(samples[0])))
    print("\nwrench mean over window (topic frame):")
    for label, (m, s) in zip(labels, stats):
        print("  %-11s % 8.3f  (std %6.3f)" % (label, m, s))

    dm = -stats[2][0] / G
    m_true = args.mass + dm
    weight = m_true * G
    r_x = stats[4][0] / weight
    r_y = -stats[3][0] / weight

    print("\n=== gravity-explainable part ===")
    print("  model mass       : %8.3f kg" % args.mass)
    print("  delta mass  dm   : %+8.3f kg  (-f_z/g)" % dm)
    print("  true mass        : %8.3f kg" % m_true)
    print("  CoG offset r_x   : %+8.1f mm  (+tau_y / m_true*g)" % (r_x * 1e3))
    print("  CoG offset r_y   : %+8.1f mm  (-tau_x / m_true*g)" % (r_y * 1e3))
    print("  (r_z unobservable at hover)")

    print("\n=== NOT explainable by gravity (geometry/servo/thrust-curve signature) ===")
    print("  f_x residual     : %+8.3f N" % stats[0][0])
    print("  f_y residual     : %+8.3f N" % stats[1][0])
    print("  tau_z residual   : %+8.3f Nm" % stats[5][0])

    if all(abs(v) < 1e-9 for v in samples[3] + samples[4] + samples[5]):
        print("\nWARNING: torque channels are all zero "
              "(torque observer disabled?) -> CoG offset invalid")
    if stats[2][1] > 2.0:
        print("\nWARNING: f_z std %.2f N is large; window is not a quiet hover, "
              "results may be biased" % stats[2][1])


if __name__ == "__main__":
    main()
