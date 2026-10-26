#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import sys
import unittest
spec = importlib.util.spec_from_file_location('hybrid', Path(__file__).parents[1]/'scripts/hybrid_motion_controller.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class DriveTests(unittest.TestCase):
    def test_zny_limits_and_wall_timeout(self):
        drive = module.HybridCommand(max_linear=1.5, max_lateral=.6, max_angular=1.)
        drive.update(2., .8, 2., 10.)
        legs, wheels = drive.output(10.49)
        self.assertEqual(legs, (0., .6, 1.))
        self.assertAlmostEqual(sum(wheels)*drive.radius/4, 1.5)
        self.assertAlmostEqual((wheels[1]-wheels[0])*drive.radius/drive.separation, 1.)
        self.assertEqual(drive.output(10.501), ((0.,0.,0.),(0.,0.,0.,0.)))

    def test_one_meter_per_second_limit(self):
        drive = module.HybridCommand()
        for requested in (1., 2., -2.):
            drive.update(requested, 0., 0., 10.)
            legs, wheels = drive.output(10.)
            expected = max(-1., min(1., requested))
            self.assertAlmostEqual(sum(wheels)*drive.radius/4, expected)
            self.assertEqual(legs, (0., 0., 0.))

    def test_velocity_split_and_order(self):
        drive = module.HybridCommand()
        drive.update(.2, .1, .3, 10.)
        legs, wheels = drive.output(10.1)
        self.assertEqual(legs, (0., .1, .3))
        self.assertAlmostEqual(wheels[0], (.2-.3*.3802/2)/.086)
        self.assertAlmostEqual(wheels[1], (.2+.3*.3802/2)/.086)
        self.assertEqual(wheels[:2], wheels[2:])

    def test_timeout_and_invalid_input_stop_both_channels(self):
        drive = module.HybridCommand()
        self.assertEqual(drive.output(0), ((0.,0.,0.),(0.,0.,0.,0.)))
        drive.update(.2, .1, .3, 10.)
        self.assertEqual(drive.output(11), ((0.,0.,0.),(0.,0.,0.,0.)))
        drive.update(.2, .1, .3, 10.)
        self.assertEqual(drive.output(9), ((0.,0.,0.),(0.,0.,0.,0.)))
        self.assertFalse(drive.update(float('nan'), 0, 0, 10))
        self.assertEqual(drive.output(10), ((0.,0.,0.),(0.,0.,0.,0.)))

    def test_joint_saturation_preserves_curvature_and_leg_yaw(self):
        drive = module.HybridCommand(max_wheel_speed=1.)
        drive.update(.3, .1, .5, 10.)
        legs, wheels = drive.output(10.)
        self.assertLessEqual(max(map(abs,wheels)),1.)
        actual_yaw=(wheels[1]-wheels[0])*.086/.3802
        self.assertAlmostEqual(legs[2],actual_yaw)
        self.assertAlmostEqual(actual_yaw/((wheels[0]+wheels[1])*.086/2), .5/.3)

    def test_invalid_parameters_fail(self):
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError): module.HybridCommand(radius=value)

if __name__=='__main__':
    unittest.main()
