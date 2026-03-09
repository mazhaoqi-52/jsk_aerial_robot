#!/usr/bin/env python
"""
Toggle unified_control_mode for beetle assembly via ROS service.

By default, auto-discovers all running beetle modules by querying the ROS
master for services matching */controller/set_unified_mode.

Usage:
  rosrun beetle set_unified_mode.py true              # enable (auto-discover)
  rosrun beetle set_unified_mode.py false             # disable (auto-discover)
  rosrun beetle set_unified_mode.py true --ids 2,3,1  # manual override
"""
import re
import sys
import rospy
import rosgraph
from std_srvs.srv import SetBool

SERVICE_PATTERN = re.compile(r'^/(.+)/controller/set_unified_mode$')


def discover_services():
    """Query ROS master for all */controller/set_unified_mode services."""
    try:
        master = rosgraph.Master('/set_unified_mode')
        _, _, services = master.getSystemState()
        matched = []
        for srv_name, _ in services:
            if SERVICE_PATTERN.match(srv_name):
                matched.append(srv_name)
        matched.sort()
        return matched
    except Exception as e:
        rospy.logwarn('Failed to query ROS master: %s', e)
        return []


def main():
    rospy.init_node('set_unified_mode', anonymous=True)

    # Parse arguments
    args = rospy.myargv(argv=sys.argv)
    if len(args) < 2 or args[1] not in ('true', 'false'):
        print(__doc__)
        sys.exit(1)

    enable = args[1] == 'true'

    # Build service list: manual --ids or auto-discover
    srv_names = []
    for i, a in enumerate(args):
        if a == '--ids' and i + 1 < len(args):
            for mid in args[i + 1].split(','):
                srv_names.append('/beetle{}/controller/set_unified_mode'.format(mid.strip()))

    if not srv_names:
        srv_names = discover_services()
        if not srv_names:
            rospy.logerr('No beetle set_unified_mode services found on ROS master')
            sys.exit(1)
        rospy.loginfo('Auto-discovered %d module(s): %s', len(srv_names),
                      [SERVICE_PATTERN.match(s).group(1) for s in srv_names])

    # Call service on each module
    success_all = True
    for srv_name in srv_names:
        name_tag = SERVICE_PATTERN.match(srv_name)
        label = name_tag.group(1) if name_tag else srv_name
        try:
            rospy.wait_for_service(srv_name, timeout=3.0)
            proxy = rospy.ServiceProxy(srv_name, SetBool)
            resp = proxy(enable)
            status = 'OK' if resp.success else 'FAIL'
            rospy.loginfo('[%s] %s  (%s)', label, resp.message, status)
            if not resp.success:
                success_all = False
        except rospy.ROSException:
            rospy.logerr('[%s] service not available (timeout)', label)
            success_all = False

    if success_all:
        rospy.loginfo('All %d modules set to unified_control_mode = %s',
                      len(srv_names), enable)
    else:
        rospy.logwarn('Some modules failed — check above')


if __name__ == '__main__':
    main()
