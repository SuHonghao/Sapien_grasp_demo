# input_utils.py

from __future__ import annotations

def key_down(viewer, key: str) -> bool:
    """
    - 优先 viewer.window.key_down('c'/'C')
    - 再 viewer.key_down('c'/'C')
    - 再 int 码 viewer.window.key_down(ord('C'))
    - 同时兼容小写/大写
    """
    cand = [key, key.lower(), key.upper()]
    objs = []
    if hasattr(viewer, "window"):
        objs.append(viewer.window)
    objs.append(viewer)

    for obj in objs:
        if hasattr(obj, "key_down"):
            for k in cand:
                try:
                    if obj.key_down(k):  # 字符版本
                        return True
                except Exception:
                    pass
                try:
                    if isinstance(k, str) and len(k) == 1:
                        if obj.key_down(ord(k)):  # 整数 key code 版本
                            return True
                except Exception:
                    pass
    return False
