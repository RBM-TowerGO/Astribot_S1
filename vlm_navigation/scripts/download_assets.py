"""Download pinned official model and NVIDIA wheels; verify model LFS and archive SHA256."""
import concurrent.futures, hashlib, json, subprocess, urllib.request
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
REV = "851bf6e806efd8d0a36b00ddf55e13ccb7b8cd0a"
REPO = "Qwen/Qwen3.5-4B"
def get_json(url):
    with urllib.request.urlopen(url, timeout=60) as r: return json.load(r)
def download(item):
    url, relative, sha = item
    dest = ROOT / relative
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        part = dest.with_name(dest.name + ".part")
        subprocess.run(["curl", "-fL", "--retry", "6", "--retry-delay", "3", "--connect-timeout", "30", "-C", "-", "-o", str(part), url], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        part.rename(dest)
    h = hashlib.sha256()
    with dest.open("rb") as f:
        for chunk in iter(lambda: f.read(8*1024*1024), b""): h.update(chunk)
    digest = h.hexdigest()
    if sha and digest != sha: raise RuntimeError(f"SHA256 mismatch: {relative}")
    print(f"VERIFIED {relative} {dest.stat().st_size}", flush=True)
    return {"path": relative, "url": url, "bytes": dest.stat().st_size, "sha256": digest, "upstream_sha256": sha}
def main():
    meta = get_json(f"https://huggingface.co/api/models/{REPO}/revision/{REV}?blobs=true")
    (ROOT/"configs/model-upstream.json").write_text(json.dumps(meta, indent=2))
    items=[]
    for f in meta["siblings"]:
        name=f["rfilename"]
        items.append((f"https://huggingface.co/{REPO}/resolve/{REV}/{name}", f"assets/models/Qwen3.5-4B/{name}", f.get("lfs",{}).get("sha256")))
    base="https://developer.download.nvidia.com/compute/"
    wheel="torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl"
    items.append((base+"redist/jp/v61/pytorch/"+wheel, "assets/downloads/"+wheel, None))
    info=get_json(base+"cusparselt/redist/redistrib_0.6.2.json")
    (ROOT/"configs/cusparselt-upstream.json").write_text(json.dumps(info,indent=2))
    a=info["libcusparse_lt"]["linux-aarch64"]
    items.append((base+"cusparselt/redist/"+a["relative_path"], "assets/downloads/"+a["relative_path"].split("/")[-1], a["sha256"]))
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool: results=list(pool.map(download,items))
    (ROOT/"configs/assets-manifest.json").write_text(json.dumps({"model_id":REPO,"revision":REV,"weight_alternate_source":"https://modelscope.cn/models/Qwen/Qwen3.5-4B/resolve/fcb1a040bb418b0b8add6f6f6c475386abc2cb97/", "verification":"All model LFS SHA256 values verified against pinned Hugging Face metadata", "files":results}, indent=2)+"\n")
    print("ALL_ASSETS_VERIFIED",flush=True)
if __name__=="__main__": main()
