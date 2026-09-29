import math,random,struct,subprocess,json
from pathlib import Path
r=random.Random(429710);cases=[]
for i in range(100000):
 exponent=r.randint(-1074,1023)
 try:values=[math.ldexp(r.uniform(-1,1),exponent+r.randint(-5,0)) for _ in range(3)]
 except OverflowError:continue
 cases.append(values)
for q in [.01,.05,.1,.5,1.-1e-6,1.+1e-6]:
 for i in range(1000):
  x=[r.uniform(-1,1) for _ in range(3)];h=math.hypot(*x);cases.append([v/h*q for v in x])
p=subprocess.run(['/tmp/review_policy_fusion/norm_probe'],input=b''.join(struct.pack('ddd',*v) for v in cases),capture_output=True,check=True)
assert len(p.stdout)==16*len(cases)
fail=[]
for v,n in zip(cases,struct.iter_unpack('dd',p.stdout)):
 for dim,got in zip([2,3],n):
  want=math.hypot(*v[:dim])
  if got!=want:fail.append(dict(v=v,dimension=dim,want=want,got=got))
Path('/tmp/review_policy_fusion/norm_stress_final.json').write_text(json.dumps(dict(cases=len(cases),mismatches=len(fail),examples=fail[:10]),indent=2)+'\n')
print('cases',len(cases),'mismatches',len(fail),fail[:4])
