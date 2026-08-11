#!/usr/bin/env python3
import unittest

from voice_command import classify_command, pcm_rms


class VoiceCommandTest(unittest.TestCase):
    def test_exact_commands(self):
        self.assertEqual(classify_command("준비"), "준비")
        self.assertEqual(classify_command("물건 넣어 줘"), "물건 넣어줘")
        self.assertEqual(classify_command("포장 해 줘"), "포장해줘")
        self.assertEqual(classify_command("종료"), "종료")

    def test_keyword_variants(self):
        self.assertEqual(classify_command("이제 준비 해"), "준비")
        self.assertEqual(classify_command("작업 시작"), "준비")
        self.assertEqual(classify_command("물건을 넣어 주세요"), "물건 넣어줘")
        self.assertEqual(classify_command("제품을 넣어 줘"), "물건 넣어줘")
        self.assertEqual(classify_command("투입 해 줘"), "물건 넣어줘")
        self.assertEqual(classify_command("포장 시작"), "포장해줘")
        self.assertEqual(classify_command("박스 포장 해 줘"), "포장해줘")
        self.assertEqual(classify_command("끝"), "종료")
        self.assertEqual(classify_command("작업 마무리"), "종료")

    def test_unrelated_or_empty_speech(self):
        self.assertIsNone(classify_command(""))
        self.assertIsNone(classify_command("안녕하세요"))
        self.assertIsNone(classify_command("해줘"))

    def test_pcm_rms(self):
        self.assertEqual(pcm_rms(b"\x00\x00" * 10), 0)
        self.assertEqual(pcm_rms((1000).to_bytes(2, "little", signed=True) * 8),
                         1000)

if __name__ == "__main__":
    unittest.main()
