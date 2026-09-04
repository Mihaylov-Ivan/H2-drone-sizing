#!/usr/bin/env python3
from pathlib import Path
import hashlib, json

ROOT=Path(__file__).resolve().parents[1]
EXCLUDE_DIRS={'.pytest_cache','__pycache__','.git'}
EXCLUDE_FILES={'PACKAGE_MANIFEST.json'}

def sha256_file(p:Path)->str:
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()

def main()->int:
    files={}
    for p in sorted(ROOT.rglob('*')):
        if not p.is_file():
            continue
        rel=p.relative_to(ROOT)
        if rel.name in EXCLUDE_FILES or any(part in EXCLUDE_DIRS for part in rel.parts):
            continue
        files[str(rel)]={'size_bytes':p.stat().st_size,'sha256':sha256_file(p)}
    payload={
        'package':'H2AIR Integrated Model v4.0',
        'release_date':'2026-09-04',
        'fidelity':'conceptual / multidisciplinary screening',
        'file_count':len(files),
        'files':files,
    }
    (ROOT/'PACKAGE_MANIFEST.json').write_text(json.dumps(payload,indent=2)+'\n',encoding='utf-8')
    print(f"Wrote PACKAGE_MANIFEST.json for {len(files)} files")
    return 0
if __name__=='__main__': raise SystemExit(main())
