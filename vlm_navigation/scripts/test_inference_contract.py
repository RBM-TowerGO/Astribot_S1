"""Contract tests for safety-relevant rejection and deterministic resizing."""
import unittest
from PIL import Image
from inference_core import validate_output, resize_image


class ContractTests(unittest.TestCase):
    def test_valid(self):
        self.assertTrue(validate_output('{"candidate_id":"A"}', True, 'A')['passed'])

    def test_invalid_outputs(self):
        cases = [('', 'empty_output'), ('```json\n{"candidate_id":"A"}\n```','invalid_json'),
                 ('{"candidate_id":"C"}', 'invalid_candidate_schema'), ('{','invalid_json'),
                 ('{"candidate_id":"B"}', 'wrong_candidate'),
                 ('{"candidate_id":"A","extra":1}', 'invalid_candidate_schema'),
                 ('{"candidate_id":"B","candidate_id":"A"}', 'invalid_json'),
                 ('{"candidate_id":["A"]}', 'invalid_candidate_schema'),
                 ('<think>x</think>{"candidate_id":"A"}', 'thinking_output')]
        for output, error in cases:
            with self.subTest(output=output):
                result = validate_output(output, True, 'A')
                self.assertFalse(result['passed'])
                self.assertIn(error, result['errors'])

    def test_truncated_even_when_json_valid(self):
        r = validate_output('{"candidate_id":"A"}', False, 'A')
        self.assertFalse(r['passed'])
        self.assertIn('truncated_or_missing_eos', r['errors'])

    def test_resize(self):
        for original, expected in [((1600,900),(768,432)), ((900,1600),(432,768)),
                                   ((512,384),(512,384)), ((768,768),(768,768))]:
            with self.subTest(original=original):
                self.assertEqual(resize_image(Image.new('RGB', original),768).size, expected)


if __name__ == '__main__':
    unittest.main()
