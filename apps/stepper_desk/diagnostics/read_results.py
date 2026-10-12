from arduino.router_bridge import Bridge
from collections import Counter
import json
import time
bridge=Bridge()
try:
    assert bridge.connect(timeout=3), 'Router unavailable'
    deadline=time.monotonic()+75
    results=None
    while time.monotonic()<deadline:
        try:
            data=bridge.call('arduino_i2c_scan_results',timeout=1)
            if len(data)==127 and data[0]>0:
                results=data
                break
        except Exception:
            pass
        time.sleep(0.25)
    if results is None: raise RuntimeError('No completed scan received')
    addresses=[i for i,error in enumerate(results[1:],1) if error==0]
    print('Standard Arduino Wire scanner: addresses 0x01 through 0x7E')
    print('Completed scan:',results[0])
    print('Responding addresses:', ', '.join('0x%02X'%a for a in addresses) or 'none')
    print('Wire.endTransmission return codes:',dict(Counter(results[1:])))
    print('All address results:',json.dumps({f'0x{i:02X}':error for i,error in enumerate(results[1:],1)}))
finally:
    bridge.disconnect()
