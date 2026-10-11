import importlib.util,json,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('gui_import',Path(__file__).resolve().parents[1]/'scripts/install-gui-ratios.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class ImportTests(unittest.TestCase):
    def fixture(self):
        joints=[dict(zero=0.,reversed=j>0,calibrated=True) for j in range(3)]
        prefs={'q.arm.geometry.v1':json.dumps({'joints':joints}).encode(),'q.arm.poses.v1':json.dumps({'poses':[{'name':'keep'}]}).encode()}
        for a in range(3):prefs[f'q.axis{a}.microsteps']=2 if a==0 else 16;prefs[f'q.axis{a}.fullSteps']=200
        fp='|'.join(f"{j}:0.0:{str(j>0).lower()}:true:{2 if j==1 else 16}:200" for j in range(3))
        trials={name:[dict(direction=d,pulses=800,degrees=8*(1 if (j==0)==(d==0) else -1)) for d in (0,1,0)] for j,name in enumerate(module.JOINTS)}
        return dict(version=1,fingerprint=fp,trials=trials),prefs
    def test_import_preserves_poses_recomputes_ratios(self):
        data,prefs=self.fixture();data['pulses_per_degree']=[999,999,999]
        result=module.prepare(data,prefs)
        self.assertEqual(result['ratios'],[100,100,100]);self.assertEqual(result['poses'],[{'name':'keep'}])
        self.assertTrue(result['trials'][0][0]['id'])
    def test_mismatch_and_invalid_counts_rejected(self):
        data,prefs=self.fixture();data['fingerprint']='old'
        with self.assertRaises(ValueError):module.prepare(data,prefs)
        data,prefs=self.fixture();data['trials']['head'][0]['pulses']=800.5
        with self.assertRaises(ValueError):module.prepare(data,prefs)
