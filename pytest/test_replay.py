"""Example suite: golden-trace record/replay/mutate through the CLI, with a
hand-assembled image (no toolchain needed — always runs).

Image layout (base 0): an SP + reset vector table, then NOPs and a spin
loop (a bare ``BKPT`` HardFaults by design, so the program spins and the
budget halts it deterministically).
"""

import struct
import vioarm_harness as harness

SP = 0x20001000

# NOP x4; B . (spin forever).
TEXT = struct.pack("<5H", 0xBF00, 0xBF00, 0xBF00, 0xBF00, 0xE7FE)


def make_image(tmp_path, text=TEXT):
    vectors = struct.pack("<II", SP, 8 | 1)
    path = tmp_path / "replay.bin"
    path.write_bytes(vectors + text)
    return str(path)


def test_record_replay_match(tmp_path):
    image = make_image(tmp_path)
    golden = str(tmp_path / "golden.txt")
    rc, out, err = harness.record(image, board="stm32f103", budget=1_000,
                                  output=golden)
    assert rc == 0, f"record failed:\n{out}\n{err}"
    rc, out, err = harness.replay(image, golden, board="stm32f103")
    assert rc == 0, f"clean replay must MATCH:\n{out}\n{err}"
    assert "MATCH" in out


def test_replay_detects_mutation(tmp_path):
    image = make_image(tmp_path)
    golden = str(tmp_path / "golden.txt")
    rc, out, err = harness.record(image, board="stm32f103", budget=1_000,
                                  output=golden)
    assert rc == 0, f"record failed:\n{out}\n{err}"
    # A patch stands in for a mutated model: one RAM byte differs.
    rc, out, err = harness.replay(image, golden, board="stm32f103",
                                  patches=["0x20000010=0x41"])
    assert rc == 1, f"mutated replay must exit 1:\n{out}\n{err}"
    assert "diff:" in out and "snap" in out


def test_replay_detects_board_mismatch(tmp_path):
    image = make_image(tmp_path)
    golden = str(tmp_path / "golden.txt")
    rc, out, err = harness.record(image, board="stm32f103", budget=1_000,
                                  output=golden)
    assert rc == 0, f"record failed:\n{out}\n{err}"
    rc, out, err = harness.replay(image, golden, board="stm32f107")
    assert rc == 1, f"board mismatch must exit 1:\n{out}\n{err}"
    assert "board" in out
