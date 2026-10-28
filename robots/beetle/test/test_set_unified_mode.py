#!/usr/bin/env python3
"""Offline CLI and real task-interface routing checks; no ROS master or tasks."""
import ast
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


BEETLE = Path(__file__).resolve().parents[1]


class UnifiedSelectionTest(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.events = []
        self.rospy = ModuleType('rospy')
        self.rospy.myargv = lambda argv: argv
        self.rospy.init_node = Mock()
        for name in ('loginfo', 'logwarn', 'logerr', 'logwarn_throttle'):
            setattr(self.rospy, name, Mock())
        self.rospy.ROSException = RuntimeError
        self.rospy.ServiceException = RuntimeError
        self.rospy.Time = SimpleNamespace(now=lambda: 123)
        self.rospy.get_param = lambda key, default=None: self.params.get(key, default)
        self.rospy.wait_for_service = Mock(side_effect=lambda s, **kw: self.events.append(('wait', s)))

        def set_param(key, value):
            self.events.append(('param', key, value))
            self.params[key] = value
        self.rospy.set_param = Mock(side_effect=set_param)

        def proxy(service, srv_type):
            def call(enable):
                self.events.append(('call', service, enable))
                key = service.rsplit('/', 1)[0] + '/unified_allocation_method'
                return SimpleNamespace(success=True, message='allocation=' + self.params.get(key, 'configured'))
            return call
        self.rospy.ServiceProxy = Mock(side_effect=proxy)
        rosgraph = ModuleType('rosgraph')
        services = [('/beetle%d/controller/set_unified_mode' % i, []) for i in (1, 2)]
        rosgraph.Master = Mock(return_value=SimpleNamespace(getSystemState=lambda: ([], [], services)))
        srv = ModuleType('std_srvs.srv')
        srv.SetBool = object
        modules = {'rospy': self.rospy, 'rosgraph': rosgraph,
                   'std_srvs': ModuleType('std_srvs'), 'std_srvs.srv': srv}
        self.modules = patch.dict(sys.modules, modules)
        self.modules.start()
        self.addCleanup(self.modules.stop)
        spec = importlib.util.spec_from_file_location('allocation_switch', BEETLE / 'script/set_unified_mode.py')
        self.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.script)

    def run_cli(self, *args):
        with patch.object(sys, 'argv', ['set_unified_mode.py'] + list(args)):
            return self.script.main()

    def test_stage_all_modules_then_enable_without_false_edge(self):
        self.assertEqual(self.run_cli('true', '--allocation', 'pinv'), 0)
        self.assertEqual([event[0] for event in self.events],
                         ['wait', 'wait', 'param', 'param', 'call', 'call'])
        self.assertEqual([event[2] for event in self.events if event[0] == 'call'], [True, True])
        self.assertTrue(all(key.endswith('/unified_allocation_method') for key in self.params))

    def test_no_argument_restores_original_configured_behavior(self):
        self.params['/beetle1/controller/unified_allocation_method'] = 'pinv'
        self.assertEqual(self.run_cli('true'), 0)
        for i in (1, 2):
            self.assertEqual(self.params['/beetle%d/controller/unified_allocation_method' % i], 'configured')

    def test_disable_keeps_existing_false_service_only(self):
        self.assertEqual(self.run_cli('false', '--ids', '2,1'), 0)
        self.rospy.set_param.assert_not_called()
        self.assertEqual([e[2] for e in self.events if e[0] == 'call'], [False, False])

    def test_missing_service_does_not_write_parameters_or_switch_modes(self):
        self.rospy.wait_for_service.side_effect = RuntimeError('unavailable')
        self.assertEqual(self.run_cli('true', '--allocation', 'qp'), 1)
        self.rospy.set_param.assert_not_called()
        self.rospy.ServiceProxy.assert_not_called()

    def test_incomplete_known_formation_is_rejected(self):
        self.params['/beetle1/assembly_leader_id'] = 1
        self.params['/beetle2/assembly_leader_id'] = 1
        self.assertEqual(self.run_cli('true', '--ids', '1', '--allocation', 'pinv'), 1)
        self.rospy.set_param.assert_not_called()
        self.rospy.ServiceProxy.assert_not_called()

    def test_explicit_selection_requires_controller_acknowledgement(self):
        self.rospy.ServiceProxy.side_effect = lambda *a: lambda enabled: SimpleNamespace(
            success=True, message='unified mode enabled')
        self.assertEqual(self.run_cli('true', '--allocation', 'pinv'), 1)
        self.assertFalse(any(e[0] == 'call' and e[2] is False for e in self.events))

    def test_invalid_arguments_have_no_ros_side_effects(self):
        for args in [('false', '--allocation', 'pinv'), ('true', '--allocation', 'lf'),
                     ('true', '--ids', '1,1')]:
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    self.run_cli(*args)
        self.rospy.init_node.assert_not_called()

    def test_real_shared_task_interface_keeps_wrench_weights_and_unload_unified(self):
        # Compile only these actual methods, avoiding the task module's imports,
        # constructors, subscribers, and state-machine execution.
        path = BEETLE / 'script/demos/valve_rotation_demo/beetle_interface.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BeetleInterface')
        names = {'isUnifiedMode', '_publishExternalWrenchWeights', 'addExternalWrench', 'clearExternalWrench'}
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        namespace = {'rospy': self.rospy, 'Float32MultiArray': lambda: SimpleNamespace(data=[])}

        def wrench_message():
            vector = lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0)
            return SimpleNamespace(header=SimpleNamespace(stamp=0, frame_id=''),
                                   wrench=SimpleNamespace(force=vector(), torque=vector()))
        namespace['WrenchStamped'] = wrench_message
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
        routing = type('Routing', (), {name: namespace[name] for name in names})

        for method in ('qp', 'pinv'):
            with self.subTest(method=method):
                self.params['/beetle2/controller/unified_control_mode'] = True
                self.params['/beetle2/controller/unified_allocation_method'] = method
                task = routing()
                task.module_id = task.wrench_target_id = 2
                task.assembly_mode = True
                task.single_uav_wrench_mode = task._inter_auto_publish = False
                task._to_list3 = lambda values: list(values)
                for publisher in ('formation_wrench_pub', 'formation_wrench_weights_pub',
                                  'desired_ext_wrench_pub', 'desired_ext_wrench_weights_pub'):
                    setattr(task, publisher, Mock())
                task.addExternalWrench([2, 0, 0], [0, 1, 0], frame_id='fc', task_weights=[1] * 6)
                task.clearExternalWrench()
                self.assertEqual(task.formation_wrench_pub.publish.call_count, 2)
                self.assertEqual(task.formation_wrench_weights_pub.publish.call_count, 2)
                task.desired_ext_wrench_pub.publish.assert_not_called()
                task.desired_ext_wrench_weights_pub.publish.assert_not_called()
                self.assertEqual(task.formation_wrench_pub.publish.call_args.args[0].wrench.force.x, 0)


if __name__ == '__main__':
    unittest.main()
