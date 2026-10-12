import importlib.util,unittest
from pathlib import Path
from unittest.mock import patch
spec=importlib.util.spec_from_file_location('evaluation',Path(__file__).resolve().parents[1]/'scripts/evaluate-calibration-model.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
class ModelTests(unittest.TestCase):
    def test_reversal_history_is_explicit_and_model_frozen(self):
        with patch.object(e,'summarize',return_value=dict(issued_pulses=1119,signed_travel_deg=-6,quality_candidate=True)):
            row=e.model_trial(Path('head-holdout6-forward-01.jsonl'))
        self.assertTrue(row['reversal']);self.assertTrue(row['within_frozen_half_degree_tolerance'])
        self.assertAlmostEqual(row['predicted_travel_deg'],859/143.1)
    def test_continuation_does_not_subtract_reversal(self):
        with patch.object(e,'summarize',return_value=dict(issued_pulses=859,signed_travel_deg=6,quality_candidate=True)):
            row=e.model_trial(Path('head-holdout6-reverse-04.jsonl'))
        self.assertFalse(row['reversal']);self.assertTrue(row['within_frozen_half_degree_tolerance'])
    def test_bad_quality_cannot_pass_good_prediction(self):
        with patch.object(e,'summarize',return_value=dict(issued_pulses=859,signed_travel_deg=6,quality_candidate=False)):
            row=e.model_trial(Path('head-holdout6-reverse-04.jsonl'))
        self.assertFalse(row['within_frozen_half_degree_tolerance'])
