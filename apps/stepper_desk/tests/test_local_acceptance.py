import copy,importlib.util,json,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('acceptance',ROOT/'scripts/build-local-acceptance-v2.py');a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
class AcceptanceTests(unittest.TestCase):
    def setUp(self):self.data=json.loads(a.TARGET.read_text())
    def test_saved_acceptance_and_provenance_validates(self):self.assertTrue(a.validate(self.data))
    def test_extrapolation_does_not_expand_supported_region(self):
        elbow=self.data['joints']['elbow']
        self.assertEqual(elbow['proposed_local_operating_region_deg'],[51.68,65.127])
        outside=[r for r in elbow['trial_summaries'] if r.get('extrapolation')]
        self.assertEqual(len(outside),1);self.assertFalse(outside[0]['scope_eligible'])
    def test_global_claim_rejected(self):
        self.data['globally_verified']=True
        with self.assertRaises(ValueError):a.validate(self.data)
    def test_modified_error_rejected(self):
        self.data['joints']['shoulder']['trial_summaries'][0]['prediction_error_deg']=2
        with self.assertRaises(ValueError):a.validate(self.data)
    def test_all_three_observed_errors_and_original_failures_preserved(self):
        self.assertEqual(set(self.data['joints']),{'shoulder','elbow','head'})
        for joint in self.data['joints'].values():
            self.assertTrue(all(abs(row['prediction_error_deg'])<=1.5 for row in joint['trial_summaries']))
        old=json.loads((ROOT/'diagnostics/local-heldout-evaluation-2026-10-11.json').read_text())
        self.assertEqual(old['endpoint_tolerance_deg'],.5)
        self.assertFalse(old['results']['shoulder']['all_pass']);self.assertFalse(old['results']['elbow']['all_pass'])
        self.assertTrue(a.validate(self.data)) # Includes hashes of original .5-degree reports.
    def test_all_forty_source_fingerprints_agree(self):
        self.assertEqual(len(self.data['recording_sources']),40)
        fingerprints={json.loads((ROOT/item['source']).read_text().splitlines()[0])['fingerprint'] for item in self.data['recording_sources']}
        profile=json.loads((ROOT/'calibration/local-endpoint-profile-v1.json').read_text())
        self.assertEqual(fingerprints,{profile['fingerprint']})
    def test_runtime_profile_retains_narrow_scope(self):
        p=json.loads((ROOT/'calibration/local-endpoint-profile-v1.json').read_text())
        self.assertFalse(p['globally_verified']);self.assertEqual(p['max_move_deg'],5)
        self.assertEqual(p['fingerprint'].split('|')[1].split(':')[-2],'2')
