"""Offline synthetic image smoke test. No cameras, ROS or robot SDK are used."""
import argparse, json, time, hashlib, subprocess
from datetime import datetime, timezone
import importlib.metadata as metadata
from pathlib import Path
import torch, psutil
from PIL import Image, ImageDraw
from transformers import AutoProcessor, Qwen3_5ForConditionalGeneration

ROOT=Path(__file__).resolve().parents[1]
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--dtype",choices=["float16","bfloat16"],default=None)
    parser.add_argument("--output",default="reports/smoke-inference.json")
    args=parser.parse_args()
    cfg=json.loads((ROOT/"configs/inference.json").read_text())
    if args.dtype:cfg["dtype"]=args.dtype
    torch.manual_seed(0)
    assert torch.cuda.is_available(), "CUDA is required"
    model_dir=ROOT/cfg["model_dir"]
    t=time.perf_counter()
    processor=AutoProcessor.from_pretrained(model_dir,local_files_only=True,use_fast=False)
    model=Qwen3_5ForConditionalGeneration.from_pretrained(model_dir,local_files_only=True,
        dtype=getattr(torch,cfg["dtype"]),device_map=cfg["device"],attn_implementation=cfg["attn_implementation"])
    model.eval()
    torch.cuda.synchronize()
    load_seconds=time.perf_counter()-t
    results=[]
    # Identical instructions, swapped pixels: the answer must change with the image.
    for index,expected in enumerate(["A","B"]):
        im=Image.new("RGB",(512,384),"white")
        draw=ImageDraw.Draw(im)
        colors=("red","blue") if expected=="A" else ("blue","red")
        draw.rectangle((40,50,220,330),fill=colors[0])
        draw.rectangle((292,50,472,330),fill=colors[1])
        path=ROOT/f"assets/samples/red_{expected}.png"
        im.save(path)
        prompt='左边的矩形候选编号是A，右边是B。请看图选择红色的矩形。合法回答只有两个：{"candidate_id":"A"} 或 {"candidate_id":"B"}。只输出其中一个JSON对象，不要解释，禁止Markdown和代码围栏。回答的第一个字符必须是{，最后一个字符必须是}。'
        messages=[{"role":"user","content":[{"type":"image","image":im},{"type":"text","text":prompt}]}]
        template=processor.apply_chat_template(messages,tokenize=False,add_generation_prompt=True,enable_thinking=False)
        assert "</think>" in template, "Disabled-thinking template is not present"
        torch.cuda.reset_peak_memory_stats()
        start=time.perf_counter()
        inputs=processor(text=[template],images=[im],return_tensors="pt",
            images_kwargs={"min_pixels":cfg["min_pixels"],"max_pixels":cfg["max_pixels"]})
        assert "pixel_values" in inputs and inputs["pixel_values"].numel()>0
        assert "</think>" in processor.tokenizer.decode(inputs["input_ids"][0])
        input_length=inputs["input_ids"].shape[-1]
        if input_length>cfg["max_input_tokens"]:raise ValueError("Input exceeds configured limit")
        inputs=inputs.to(model.device)
        with torch.inference_mode():
            output=model.generate(**inputs,max_new_tokens=cfg["max_new_tokens"],do_sample=False)
        torch.cuda.synchronize()
        elapsed=time.perf_counter()-start
        generated=output[0,input_length:]
        text=processor.decode(generated,skip_special_tokens=True).strip()
        try:parsed=json.loads(text)
        except json.JSONDecodeError:parsed=None
        eos=model.generation_config.eos_token_id
        eos=eos if isinstance(eos,list) else [eos]
        ended=int(generated[-1]) in eos
        ok=parsed=={"candidate_id":expected} and ended and "<think>" not in text
        result={"sample":str(path.relative_to(ROOT)),"sample_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"expected":expected,"output":text,
            "passed":ok,"eos_reached":ended,"input_tokens":input_length,"output_tokens":len(generated),
            "decision_seconds":elapsed,"cuda_peak_allocated_bytes":torch.cuda.max_memory_allocated(),
            "cuda_peak_reserved_bytes":torch.cuda.max_memory_reserved(),"process_rss_bytes":psutil.Process().memory_info().rss}
        results.append(result)
        print(json.dumps(result,ensure_ascii=False),flush=True)
    report={"test":"synthetic image-conditioning smoke; not navigation accuracy or benchmark",
        "timestamp_utc":datetime.now(timezone.utc).isoformat(),
        "git_head":subprocess.check_output(["git","-C",str(ROOT),"rev-parse","HEAD"],text=True).strip(),
        "git_dirty":bool(subprocess.check_output(["git","-C",str(ROOT),"status","--porcelain"],text=True).strip()),
        "power_mode":subprocess.check_output(["/usr/sbin/nvpmodel","-q"],text=True).strip(),
        "gpu":torch.cuda.get_device_name(0),"transformers":metadata.version("transformers"),
        "config":cfg,"load_seconds":load_seconds,"torch":torch.__version__,"results":results,
        "all_passed":all(x["passed"] for x in results)}
    (ROOT/args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
    if not report["all_passed"]:raise SystemExit("Smoke test failed; see report")
if __name__=="__main__":main()
