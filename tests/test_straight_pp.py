"""straight_pp_k: straight-mode pure-pursuit blend; 0 = off and byte-identical."""
import os, re
SRC = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src', 'pit_crew', 'mpc3_core.py')).read()


def test_off_by_default():
    assert re.search(r"straight_pp_k=0\.0,", SRC)


def test_blend_uses_the_straight_gate_and_runs_before_the_publish_gate():
    a = SRC.index("spk = float(c.get('straight_pp_k'")
    b = SRC.index("gc = float(c.get('pub_gate_corr'")
    blk = SRC[a:b]
    assert a < b and "_straight_gate(i0, e_lat)" in blk
    assert "(1.0 - gpp) * cmd + gpp * pp" in blk and "-cap, cap" in blk
