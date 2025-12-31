# Example Characters

This directory contains example Chinese character stroke data in JSON format.

## Format

Each character file contains:
- `character`: The Chinese character
- `pinyin`: Romanization
- `meaning`: English meaning
- `stroke_count`: Number of strokes
- `strokes`: Array of stroke data
  - `stroke_number`: Order of the stroke
  - `type`: Type of stroke (horizontal, vertical, etc.)
  - `points`: Array of [x, y] coordinates in normalized space (0-200)

## Available Characters

- **一 (yi)**: "one" - Single horizontal stroke
- **十 (shi)**: "ten" - Cross (vertical + horizontal)
- **木 (mu)**: "tree/wood" - Tree character with 4 strokes

## Adding New Characters

Create a new JSON file following this template:

```json
{
  "character": "字",
  "pinyin": "zi",
  "meaning": "character",
  "stroke_count": N,
  "strokes": [
    {
      "stroke_number": 1,
      "type": "stroke_type",
      "points": [
        [x1, y1],
        [x2, y2],
        ...
      ]
    }
  ]
}
```

### Coordinate System

- Origin (0, 0) is top-left
- X increases to the right
- Y increases downward
- Typical character fits in 200x200 space
- Add margin around character edges

### Stroke Types

Common stroke types:
- `horizontal`: 横 (héng)
- `vertical`: 竖 (shù)
- `left-falling`: 撇 (piě)
- `right-falling`: 捺 (nà)
- `dot`: 点 (diǎn)
- `hook`: 钩 (gōu)
- `rising`: 提 (tí)

## Usage

```python
from software.calligraphy import CalligraphyPlanner

planner = CalligraphyPlanner()
planner.load_character('examples/characters/yi.json')
```

## Resources

For more characters, consider:
- Makemeahanzi project: https://github.com/skishore/makemeahanzi
- Hanzi Writer: https://github.com/chanind/hanzi-writer-data
