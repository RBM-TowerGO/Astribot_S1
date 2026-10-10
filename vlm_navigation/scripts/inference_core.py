"""Shared offline inference. No ROS, camera, robot SDK or motion dependencies."""
import hashlib
import io
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROMPT = '左边的矩形候选编号是A，右边是B。请看图选择红色的矩形。合法回答只有两个：{"candidate_id":"A"} 或 {"candidate_id":"B"}。只输出其中一个JSON对象，不要解释，禁止Markdown和代码围栏。回答的第一个字符必须是{，最后一个字符必须是}。'


def read_config():
    return json.loads((ROOT / 'configs/inference.json').read_text())


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key')
        result[key] = value
    return result


def validate_output(text, eos_reached, expected):
    errors = []
    if not eos_reached:
        errors.append('truncated_or_missing_eos')
    if not text.strip():
        errors.append('empty_output')
    if '<think>' in text or '</think>' in text:
        errors.append('thinking_output')
    try:
        parsed = json.loads(text, object_pairs_hook=_unique_object)
        json_valid = True
    except (ValueError, TypeError):
        parsed = None
        json_valid = False
        errors.append('invalid_json')
    schema_valid = (isinstance(parsed, dict) and set(parsed) == {'candidate_id'}
                    and isinstance(parsed['candidate_id'], str)
                    and parsed['candidate_id'] in ('A', 'B'))
    if json_valid and not schema_valid:
        errors.append('invalid_candidate_schema')
    correct = schema_valid and parsed['candidate_id'] == expected
    if schema_valid and not correct:
        errors.append('wrong_candidate')
    return dict(passed=not errors, errors=errors, json_valid=json_valid,
                schema_valid=schema_valid, correct=correct)


def resize_image(image, long_edge):
    if long_edge <= 0:
        raise ValueError('image_long_edge must be positive')
    if max(image.size) <= long_edge:
        return image
    from PIL import Image
    ratio = long_edge / max(image.size)
    size = tuple(max(1, round(n * ratio)) for n in image.size)
    return image.resize(size, Image.Resampling.LANCZOS)


class InputLimitError(ValueError):
    def __init__(self, actual, limit):
        self.actual, self.limit = actual, limit
        super().__init__(f'Input tokens {actual} exceed limit {limit}')


class InferenceEngine:
    def __init__(self, config=None):
        import torch
        from transformers import AutoProcessor, Qwen3_5ForConditionalGeneration
        self.torch = torch
        self.config = config or read_config()
        c = self.config
        if c['enable_thinking'] is not False or c['do_sample'] is not False:
            raise ValueError('P1 requires thinking disabled and deterministic decoding')
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is required')
        torch.manual_seed(0)
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        start = time.perf_counter()
        model_dir = ROOT / c['model_dir']
        self.processor = AutoProcessor.from_pretrained(model_dir, local_files_only=True, use_fast=False)
        self.model = Qwen3_5ForConditionalGeneration.from_pretrained(
            model_dir, local_files_only=True, dtype=getattr(torch, c['dtype']),
            device_map=c['device'], attn_implementation=c['attn_implementation'])
        self.model.eval()
        torch.cuda.synchronize()
        self.load_seconds = time.perf_counter() - start
        self.load_memory = self.memory()

    def memory(self):
        t = self.torch
        return dict(cuda_peak_allocated_bytes=t.cuda.max_memory_allocated(),
                    cuda_peak_reserved_bytes=t.cuda.max_memory_reserved())

    def infer(self, sample, expected, *, max_new_tokens=None, max_input_tokens=None):
        from PIL import Image
        import psutil
        t, c = self.torch, self.config
        path = Path(sample)
        if not path.is_absolute():
            path = ROOT / path
        t.cuda.synchronize()
        t.cuda.reset_peak_memory_stats()
        start = time.perf_counter()
        result = dict(sample=str(path), expected=expected)
        try:
            raw = path.read_bytes()
            with Image.open(io.BytesIO(raw)) as source:
                im = source.convert('RGB')
            result['original_size'] = list(im.size)
            im = resize_image(im, c['image_long_edge'])
            result['processed_size'] = list(im.size)
            result['sample_sha256'] = hashlib.sha256(raw).hexdigest()
            messages = [{'role':'user', 'content':[{'type':'image','image':im}, {'type':'text','text':PROMPT}]}]
            template = self.processor.apply_chat_template(messages, tokenize=False,
                add_generation_prompt=True, enable_thinking=False)
            if '</think>' not in template:
                raise ValueError('Disabled-thinking template missing')
            inputs = self.processor(text=[template], images=[im], return_tensors='pt',
                images_kwargs={'min_pixels':c['min_pixels'], 'max_pixels':c['max_pixels']})
            if 'pixel_values' not in inputs or inputs['pixel_values'].numel() == 0:
                raise ValueError('Visual input missing')
            if '</think>' not in self.processor.tokenizer.decode(inputs['input_ids'][0]):
                raise ValueError('Disabled-thinking input tokens missing')
            length = inputs['input_ids'].shape[-1]
            result['input_tokens'] = length
            result['image_grid_thw'] = inputs['image_grid_thw'].tolist()
            result['pixel_values_shape'] = list(inputs['pixel_values'].shape)
            limit = c['max_input_tokens'] if max_input_tokens is None else max_input_tokens
            if length > limit:
                raise InputLimitError(length, limit)
            budget = c['max_new_tokens'] if max_new_tokens is None else max_new_tokens
            if budget <= 0 or budget > c['max_new_tokens']:
                raise ValueError('Invalid generation budget')
            inputs = inputs.to(self.model.device)
            with t.inference_mode():
                output = self.model.generate(**inputs, max_new_tokens=budget, do_sample=False)
            t.cuda.synchronize()
            generated = output[0, length:]
            text = self.processor.decode(generated, skip_special_tokens=True).strip()
            eos = self.model.generation_config.eos_token_id
            eos = eos if isinstance(eos, list) else [eos]
            ended = bool(len(generated)) and int(generated[-1]) in eos
            result.update(output=text, output_tokens=len(generated), eos_reached=ended,
                          max_new_tokens=budget, **validate_output(text, ended, expected))
        except InputLimitError as exc:
            result.update(passed=False, errors=['input_limit_exceeded'], input_tokens=exc.actual,
                          max_input_tokens=exc.limit, generation_started=False)
        t.cuda.synchronize()
        result['decision_seconds'] = time.perf_counter() - start
        result.update(self.memory(), process_rss_bytes=psutil.Process().memory_info().rss)
        return result
