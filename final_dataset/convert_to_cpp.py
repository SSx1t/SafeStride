"""Convert the trained sklearn model into a C++ header for the ESP32."""
import joblib, os
from micromlgen import port

clf = joblib.load('safestride_model.pkl')
c_code = port(clf, classname='GaitClassifier')

with open('gait_model.h', 'w') as f:
    f.write('#ifndef GAIT_MODEL_H\n#define GAIT_MODEL_H\n\n')
    f.write('// Auto-generated. DO NOT EDIT.\n\n')
    f.write(c_code)
    f.write('\n#endif\n')

print(f"gait_model.h: {os.path.getsize('gait_model.h')} bytes")