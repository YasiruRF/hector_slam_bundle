#!/usr/bin/env python3
"""Place top-level windows on a WM-less X display (Xvfb) and list them.

  xwin.py list
  xwin.py move PATTERN X Y W H     first top-level window whose title contains PATTERN
  xwin.py raise PATTERN
Uses DISPLAY; needs python-xlib.
"""
import sys

from Xlib import X, display


def toplevels(d):
    out = []
    for w in d.screen().root.query_tree().children:
        try:
            name = w.get_wm_name() or ''
            g = w.get_geometry()
            mapped = w.get_attributes().map_state == X.IsViewable
        except Exception:
            continue
        out.append((w, str(name), g, mapped))
    return out


def find(d, pattern):
    best = None
    for w, name, g, mapped in toplevels(d):
        if pattern.lower() in name.lower() and mapped:
            if best is None or g.width * g.height > best[1]:
                best = (w, g.width * g.height)
    if best is None:
        sys.exit('no window matching %r' % pattern)
    return best[0]


def main():
    d = display.Display()
    cmd = sys.argv[1]
    if cmd == 'list':
        for w, name, g, mapped in toplevels(d):
            if mapped and g.width > 50:
                print('0x%x %dx%d+%d+%d %s' % (w.id, g.width, g.height, g.x, g.y, name))
    elif cmd == 'move':
        x, y, ww, hh = map(int, sys.argv[3:7])
        w = find(d, sys.argv[2])
        w.configure(x=x, y=y, width=ww, height=hh, stack_mode=X.Above)
    elif cmd == 'raise':
        find(d, sys.argv[2]).configure(stack_mode=X.Above)
    d.sync()


if __name__ == '__main__':
    main()
