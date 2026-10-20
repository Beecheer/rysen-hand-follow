"""Resumable ranged download from the official repository; explicit proxy only."""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib
from pathlib import Path
import subprocess
import shutil

ROOT=Path(__file__).resolve().parent/'models'/'pretrained_models'
BASE='https://huggingface.co/warmshao/WiLoR-mini/resolve/main/pretrained_models/'
SHA='3e97aafc7dd08d883a4cc5a027df61fdb6fda6136dbd1319405413862ada6bb2'
SIZE=2564989533

def main():
    p=argparse.ArgumentParser();p.add_argument('--proxy');p.add_argument('--torch',action='store_true');args=p.parse_args()
    global ROOT,BASE,SHA,SIZE
    if args.torch:
        ROOT=Path(__file__).resolve().parent/'wheels'
        BASE='https://download.pytorch.org/whl/cu121/'
        SHA='5eb9763bec4c9c4d0194f6a01cdb9d46cf8c924c74d4bd92e1b7db6a7e532bed'
        SIZE=2449398960
    ROOT.mkdir(parents=True,exist_ok=True)
    common=['curl.exe','--fail','--location','--retry','3','--connect-timeout','20','--max-time','240','--silent','--show-error']
    if args.proxy:common+=['--proxy',args.proxy]
    name='torch-2.5.0+cu121-cp310-cp310-win_amd64.whl' if args.torch else 'wilor_final.ckpt';dest=ROOT/name
    if not dest.exists():
        parts=ROOT/'parts';parts.mkdir(exist_ok=True)
        if not args.torch and (ROOT/'wilor-parts').exists():parts=ROOT/'wilor-parts'
        chunk=16*1024*1024
        def download(index):
            start=index*chunk;end=min(SIZE,start+chunk)-1;path=parts/f'{index:04}.part'
            if not path.exists() or path.stat().st_size!=end-start+1:
                source=BASE+name.replace('+','%2B')+f'?download=true&part={index}'
                subprocess.run(common+['--range',f'{start}-{end}','--output',str(path),source],check=True)
            if path.stat().st_size!=end-start+1:raise RuntimeError(f'Unexpected range size {index}')
            return index
        count=(SIZE+chunk-1)//chunk
        with ThreadPoolExecutor(max_workers=8) as pool:
            futures=[pool.submit(download,i) for i in range(count)]
            for done,f in enumerate(as_completed(futures),1):
                f.result()
                if done%8==0 or done==count:print(f'Model chunks {done}/{count}',flush=True)
        temp=dest.with_suffix('.download')
        with temp.open('wb') as out:
            for i in range(count):
                with (parts/f'{i:04}.part').open('rb') as src:shutil.copyfileobj(src,out)
        digest=hashlib.sha256()
        with temp.open('rb') as src:
            for block in iter(lambda:src.read(8*1024*1024),b''):digest.update(block)
        if digest.hexdigest()!=SHA:raise RuntimeError('WiLoR SHA256 mismatch')
        temp.replace(dest)
        print(f'{name} SHA256 verified',flush=True)
    if args.torch:return
    for name in ['detector.pt','mano_mean_params.npz','MANO_RIGHT.pkl']:
        dest=ROOT/name
        if dest.exists():continue
        temp=dest.with_suffix(dest.suffix+'.download')
        subprocess.run(common+['--output',str(temp),BASE+name],check=True)
        temp.replace(dest);print(f'Downloaded {name}',flush=True)
    expected={'wilor_final.ckpt':SHA,
        'detector.pt':'5ef3df44e42d2db52d4ffe91f83a22ce9925e2acc9abebf453f2c5d22e380033',
        'MANO_RIGHT.pkl':'45d60aa3b27ef9107a7afd4e00808f307fd91111e1cfa35afd5c4a62de264767',
        'mano_mean_params.npz':'efc0ec58e4a5cef78f3abfb4e8f91623b8950be9eff8b8e0dbb0d036ebc63988'}
    for name,digest in expected.items():
        hasher=hashlib.sha256()
        with (ROOT/name).open('rb') as src:
            for block in iter(lambda:src.read(8*1024*1024),b''):hasher.update(block)
        if hasher.hexdigest()!=digest:raise RuntimeError(f'{name} SHA256 mismatch')
    print('All four official LFS SHA256 hashes verified',flush=True)

if __name__=='__main__':main()
