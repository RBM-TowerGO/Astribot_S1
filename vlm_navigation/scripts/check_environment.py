"""Validate the isolated Jetson runtime without loading weights or robot SDK."""
import importlib.metadata as md, json, site, sys, argparse
from datetime import datetime, timezone
from pathlib import Path
import torch, torchvision
from transformers import AutoProcessor, Qwen3_5ForConditionalGeneration
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
assert not site.ENABLE_USER_SITE
assert torch.cuda.is_available()
x=torch.randn((256,256),dtype=torch.float16,device="cuda")
assert torch.isfinite(x@x.T).all()
boxes=torch.tensor([[0,0,10,10],[1,1,9,9]],dtype=torch.float32,device="cuda")
scores=torch.tensor([0.9,0.8],device="cuda")
assert torchvision.ops.nms(boxes,scores,0.5).tolist()==[0]
p=AutoProcessor.from_pretrained(ROOT/"assets/models/Qwen3.5-4B",local_files_only=True,use_fast=False)
messages=[{"role":"user","content":[{"type":"image","image":Image.new("RGB",(384,384),"red")},{"type":"text","text":"Which color?"}]}]
template=p.apply_chat_template(messages,tokenize=False,add_generation_prompt=True,enable_thinking=False)
assert "</think>" in template
inputs=p(text=[template],images=[messages[0]["content"][0]["image"]],return_tensors="pt")
assert "</think>" in p.tokenizer.decode(inputs["input_ids"][0])
assert inputs["pixel_values"].numel()>0
torch.cuda.synchronize()
report={"python":sys.version,"executable":sys.executable,"user_site_disabled":not site.ENABLE_USER_SITE,
    "versions":{name:md.version(name) for name in ["torch","torchvision","transformers","accelerate","numpy","Pillow"]},
    "cuda":torch.version.cuda,"gpu":torch.cuda.get_device_name(0),"capability":torch.cuda.get_device_capability(0),
    "fp16_matmul":True,"torchvision_cuda_nms":True,"processor":type(p).__name__,
    "image_tensor_shape":list(inputs["pixel_values"].shape),"thinking_disabled_template":True,
    "scope":"Environment and preprocessing only; no weights loaded, no robot action"}
parser=argparse.ArgumentParser()
parser.add_argument("--output")
args=parser.parse_args()
output=Path(args.output) if args.output else ROOT/"reports"/("environment-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ"))/"environment-check.json"
if not output.is_absolute():output=ROOT/output
output.parent.mkdir(parents=True,exist_ok=True)
if output.exists():raise FileExistsError(output)
output.write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))
