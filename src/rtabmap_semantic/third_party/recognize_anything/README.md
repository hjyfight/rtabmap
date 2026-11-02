# Recognize Anything bilingual tag vocabulary

Source: https://github.com/xinyu1205/recognize-anything

Files: `ram/data/ram_tag_list.txt`, `ram/data/ram_tag_list_chinese.txt` (main, downloaded 2026-11-02).

License: Apache-2.0, reproduced in LICENSE.

Converted matching English/Chinese lines into `rtabmap_semantic/class_names_zh.py`; stripped surrounding whitespace and lowercased English lookup keys. All 4,585 English names were verified in order against the installed official YOLOE-11s-seg-pf checkpoint. Local display corrections and category normalization are in `labels.py`. No images are sent to a translation service.

Source file SHA-256:

- `rtabmap_ram_tag_list_en.txt`: `5d96d2f7225527480ad773aaac76b6e5c08a18adf6eae9ca7d2ab923d5e7bd8b`
- `rtabmap_ram_tag_list_zh.txt`: `c912657a0f30be7b2f5df6ccffa17e1d84561d0bea508528e38b6d0d711bcfbe`
