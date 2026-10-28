#!/usr/bin/env python
"""
Toggle unified_control_mode for beetle assembly via ROS service.

By default, auto-discovers all running beetle modules by querying the ROS
master for services matching */controller/set_unified_mode.

Usage:
  rosrun beetle set_unified_mode.py true              # enable (auto-discover)
  rosrun beetle set_unified_mode.py false             # disable (auto-discover)
  rosrun beetle set_unified_mode.py true --ids 2,3,1  # manual override
  rosrun beetle set_unified_mode.py true --allocation qp
  rosrun beetle set_unified_mode.py true --allocation pinv

Without --allocation, use the original bringup allocation settings. Both qp
and pinv remain unified mode; no task, yaw, or actuator settings are changed.
"""
import argparse
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('enable', choices=('true', 'false'))
    parser.add_argument('--ids', help='Comma-separated module IDs (all formation members)')
    parser.add_argument('--allocation', choices=('qp', 'pinv'), default=None)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    enable = args.enable == 'true'
    if not enable and args.allocation is not None:
        parser.error('--allocation is only valid with true')

    # Build service list: manual --ids or auto-discover
    srv_names = []
    if args.ids is not None:
        ids = [mid.strip() for mid in args.ids.split(',')]
        if (not all(re.match(r'^[1-9][0-9]*$', mid) for mid in ids) or
                len(set(ids)) != len(ids)):
            parser.error('--ids must contain distinct positive integers')
        srv_names = ['/beetle{}/controller/set_unified_mode'.format(mid) for mid in ids]

    rospy.init_node('set_unified_mode', anonymous=True)

    if not srv_names:
        srv_names = discover_services()
        if not srv_names:
            rospy.logerr('No beetle set_unified_mode services found on ROS master')
            return 1
        rospy.loginfo('Auto-discovered %d module(s): %s', len(srv_names),
                      [SERVICE_PATTERN.match(s).group(1) for s in srv_names])

    # Preflight every service before staging any selection or calling a service.
    # Selection is applied synchronously by that SAME existing service callback;
    # setParam alone does not reconfigure the running allocator.
    try:
        for srv_name in srv_names:
            rospy.wait_for_service(srv_name, timeout=3.0)
        if enable:
            selected = {s.rsplit('/controller/', 1)[0] for s in srv_names}
            if args.allocation is not None and args.ids is not None:
                # Navigator actually publishes assembly_leader_id. Do not rely
                # on assembly_ids, which older launches may never populate.
                leaders = {rospy.get_param(ns + '/assembly_leader_id', None) for ns in selected}
                leaders.discard(None)
                missing = []
                for service in discover_services():
                    ns = service.rsplit('/controller/', 1)[0]
                    if ns not in selected and rospy.get_param(ns + '/assembly_leader_id', None) in leaders:
                        missing.append(ns)
                if missing:
                    rospy.logerr('Selection omits discovered members of the same formation: %s', missing)
                    return 1
            method = args.allocation or 'configured'
            for srv_name in srv_names:
                parameter = srv_name.rsplit('/', 1)[0] + '/unified_allocation_method'
                rospy.set_param(parameter, method)
    except Exception as exc:
        rospy.logerr('Preflight/staging failed; no mode services called: %s', exc)
        return 1

    # Do not toggle false/true to change allocator. Tasks continue to see unified.
    success_all = True
    for srv_name in srv_names:
        name_tag = SERVICE_PATTERN.match(srv_name)
        label = name_tag.group(1) if name_tag else srv_name
        try:
            proxy = rospy.ServiceProxy(srv_name, SetBool)
            resp = proxy(enable)
            status = 'OK' if resp.success else 'FAIL'
            rospy.loginfo('[%s] %s  (%s)', label, resp.message, status)
            if not resp.success:
                success_all = False
            elif enable and args.allocation is not None and 'allocation=' + method not in resp.message:
                rospy.logerr('[%s] controller did not acknowledge allocation=%s; '
                             'check that all modules use the updated controller', label, method)
                success_all = False
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr('[%s] service failed: %s', label, exc)
            success_all = False

    if success_all:
        rospy.loginfo('All %d modules set to unified_control_mode = %s',
                      len(srv_names), enable)
    else:
        rospy.logwarn('Some modules failed — check above')
    return 0 if success_all else 1


if __name__ == '__main__':
    sys.exit(main())
