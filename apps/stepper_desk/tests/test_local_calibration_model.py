import importlib.util,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('localfit',Path(__file__).resolve().parents[1]/'scripts/fit-local-calibration-model.py')
f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)
class LocalModelTests(unittest.TestCase):
    def test_crossing_knot_both_directions(self):
        # From -2 to +3: 2*100+3*200=800 pulses plus 50 reversal.
        self.assertAlmostEqual(f.predict(-2,850,1,True,[0,100,200,50]),3)
        self.assertAlmostEqual(f.predict(3,800,-1,False,[0,100,200,50]),-2)
    def test_known_normal_equations(self):
        result=f.solve([[2,1],[1,3]],[5,7])
        self.assertAlmostEqual(result[0],1.6);self.assertAlmostEqual(result[1],1.8)
