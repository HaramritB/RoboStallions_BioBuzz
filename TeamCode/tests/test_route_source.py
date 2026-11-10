"""Source-of-truth and malformed-input checks for the actual route generator/auditor."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('route_sync',ROOT/'TeamCode/tools/sync_autonomous_route.py')
route=importlib.util.module_from_spec(spec)
spec.loader.exec_module(route)

class RouteSourceChecks(unittest.TestCase):
    def setUp(self):
        self.raw=route.SOURCE.read_bytes()
        self.data=json.loads(self.raw)
    def generate(self,data):
        return route.generate(json.dumps(data,allow_nan=True).encode())
    def test_generated_files_match_source(self):
        java,audit=route.generate(self.raw)
        self.assertEqual(java,route.JAVA.read_text())
        self.assertEqual(audit,route.AUDIT.read_text())
        self.assertEqual(json.loads(audit)['source_sha256'],hashlib.sha256(self.raw).hexdigest())
    def test_actual_conflict_and_missing_measurements(self):
        _,audit=route.generate(self.raw); audit=json.loads(audit)
        self.assertFalse(audit['robot_dimensions_present'])
        self.assertEqual(audit['path4_center_to_reference_upper_boundary_in'],1.2695)
        self.assertEqual([hit['path'] for hit in audit['center_conflicts']],[4,5])
        self.assertIn('Scoring-Side FLOWER',audit['center_conflicts'][0]['endpoint_inside'])
        self.assertEqual(audit['explicit_turns'][0]['before_path'],3)
    def test_reordered_actions_are_not_silently_accepted(self):
        self.data['sequence'].reverse()
        with self.assertRaises(ValueError): self.generate(self.data)
    def test_tangent_cannot_override_requested_headings(self):
        self.data['lines'][0]['heading']={'type':'tangential','reverse':False}
        with self.assertRaises(ValueError): self.generate(self.data)
    def test_nonfinite_coordinate_rejected(self):
        self.data['lines'][1]['controlPoints'][0]['x']=float('nan')
        with self.assertRaises(ValueError): self.generate(self.data)
    def test_initial_heading_must_match_first_path(self):
        self.data['startPoint']['headingDeg']=0
        with self.assertRaises(ValueError): self.generate(self.data)
    def test_zero_length_cannot_encode_turn(self):
        self.data['lines'][0]['endPoint'].update(x=self.data['startPoint']['x'],y=self.data['startPoint']['y'])
        with self.assertRaises(ValueError): self.generate(self.data)
    def test_explicit_synthetic_footprint_is_audited(self):
        # Synthetic dimensions exercise the algorithm; they are NOT BioBuzz measurements.
        self.data['robot']={'rLength':14,'rWidth':14}
        _,audit=self.generate(self.data); audit=json.loads(audit)
        self.assertTrue(audit['robot_dimensions_present'])
        self.assertTrue(any(hit['path']==4 for hit in audit['sampled_footprint_conflicts']))
        self.assertTrue(audit['clearance_status'].startswith('BLOCKED'))

if __name__=='__main__': unittest.main()
