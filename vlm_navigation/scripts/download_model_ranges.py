"""Resume bounded parallel HTTP ranges and verify full official LFS SHA256."""
import concurrent.futures, hashlib, json, os, time, urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
META=json.loads((ROOT/"configs/model-upstream.json").read_text())
REV=META["sha"]
CHUNK=16*1024*1024

def fetch(task):
    name,start,end,total=task
    path=ROOT/"assets/downloads/ranges"/name/str(start)
    path.parent.mkdir(parents=True,exist_ok=True)
    length=end-start+1
    if path.exists() and path.stat().st_size==length:return
    url=f"https://modelscope.cn/models/Qwen/Qwen3.5-4B/resolve/fcb1a040bb418b0b8add6f6f6c475386abc2cb97/{name}?range_start={start}"
    for attempt in range(6):
        try:
            partial=path.with_suffix(".part")
            have=partial.stat().st_size if partial.exists() else 0
            if have==length:
                partial.rename(path)
                return
            offset=start+have
            req=urllib.request.Request(url+f"&offset={offset}",headers={"Range":f"bytes={offset}-{end}"})
            with urllib.request.urlopen(req,timeout=90) as resp:
                expected=f"bytes {offset}-{end}/{total}"
                if resp.status!=206 or resp.headers.get("Content-Range")!=expected:raise RuntimeError("Unexpected range response")
                with path.with_suffix(".part").open("ab") as f:
                    while data:=resp.read(1024*1024):f.write(data)
            if path.with_suffix(".part").stat().st_size!=length:raise RuntimeError("Incomplete range")
            path.with_suffix(".part").rename(path)
            return
        except Exception:
            if attempt==5:raise
            time.sleep(min(2**attempt,20))
def main():
    files=[f for f in META["siblings"] if f["rfilename"].endswith(".safetensors")]
    tasks=[(f["rfilename"],s,min(s+CHUNK,f["size"])-1,f["size"]) for f in files for s in range(0,f["size"],CHUNK)]
    t=time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(max_workers=32) as pool:
        for n,_ in enumerate(pool.map(fetch,tasks),1):
            if n%8==0:print(f"RANGES {n}/{len(tasks)} elapsed={time.monotonic()-t:.1f}s",flush=True)
    for f in files:
        name=f["rfilename"];dest=ROOT/"assets/models/Qwen3.5-4B"/name
        h=hashlib.sha256()
        with dest.with_suffix(".assembled").open("wb") as out:
            for start in range(0,f["size"],CHUNK):
                data=(ROOT/"assets/downloads/ranges"/name/str(start)).read_bytes();h.update(data);out.write(data)
        if h.hexdigest()!=f["lfs"]["sha256"]:raise RuntimeError(f"Hash mismatch: {name}")
        dest.with_suffix(".assembled").rename(dest)
        print(f"VERIFIED {name} SHA256={h.hexdigest()}",flush=True)
    print("MODEL_COMPLETE",flush=True)
if __name__=="__main__":main()
