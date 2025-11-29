#!/usr/bin/env python3
# LLM
import sys
import os
import textwrap

TEMPLATE = """package demos

//////////////////////////////////////////////// {Name}
// (oimo/path)
// {Desc}

type {Name} struct {{
\t// TODO
}}

func New{Name}() *{Name} {{
\treturn &{Name}{{}}
}}

// TODO
"""

def main(argv):
    if len(argv) < 2 or len(argv) > 3:
        print("Usage: python objgen.py Name \"Description\"")
        sys.exit(2)

    name = argv[1].strip()
    desc = argv[2].strip() if len(argv) == 3 else ""

    if not name.isidentifier():
        print("Error: Name must be a valid identifier.")
        sys.exit(2)

    filename = f"demos/{name}.go"
    content = TEMPLATE.format(Name=name, Desc=desc)

    # Avoid overwriting existing file accidentally
    if os.path.exists(filename):
        print(f"Error: '{filename}' already exists. Remove it first or choose a different name.")
        sys.exit(1)

    with open(filename, "w", newline="\n") as f:
        f.write(content)

    print(f"Wrote {filename}")

if __name__ == "__main__":
    main(sys.argv)
