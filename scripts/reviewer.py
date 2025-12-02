#!/usr/bin/env python3
# partial LLM
# requirements: pip3 install google-genai rich
# markdown: dnf install glow

import argparse
import json
import shutil
import shlex
import subprocess
import os

from google import genai
from google.genai.types import Tool, GenerateContentConfig, HarmCategory, SafetySetting

SETTINGS_FILENAME = os.path.expanduser("~/.OimoPhysics.reviewer")
REVIEW_FILENAME = "review.result.md"
OIMO_PHYSICS_PATH = os.path.expanduser("~/dev/OimoPhysics")


def configure():
    bash_cmd = f"source {shlex.quote(SETTINGS_FILENAME)} && env"
    result = subprocess.run(
        ["bash", "-c", bash_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True,
    )


def load_file(path):
    with open(path, "r") as f:
        return f.read()


def write_output(path, content):
    with open(path, "w") as f:
        f.write(content)


def print_markdown(review_path):
    glow_path = shutil.which("glow")
    if glow_path is None:
        print(review)
    try:
        subprocess.run([glow_path, review_path], check=True)
    except subprocess.CalledProcessError as e:
        print(review)


def find_code_file(search_path, base_name, extension):
    """
    Recursively searches for a file with the given base name and extension.

    Raises:
        FileNotFoundError: If no matching file is found.
    """
    target_filename = base_name + extension

    for dirpath, dirnames, filenames in os.walk(search_path):
        if target_filename in filenames:
            return os.path.join(dirpath, target_filename)

    raise FileNotFoundError(f"File '{target_filename}' not found under '{search_path}'")


def main():
    parser = argparse.ArgumentParser(
        description="Review Go↔Haxe conversion using Gemini 2.5 Pro."
    )
    parser.add_argument(
        "basename",
        help="Object name to review, searches for both Go (in current path) and Haxe (in configured OimoPhysics path)",
    )
    args = parser.parse_args()

    try:
        configure()
    except subprocess.CalledProcessError as e:
        print("Failed to execute environment script:")
        print(e.stderr)
        return False

    if not "GOOGLE_API_KEY" in os.environ:
        print("Google Gemini api key not set: GOOGLE_API_KEY")
        return False

    go_file = ""
    haxe_file = ""
    try:
        go_file = find_code_file(".", args.basename, ".go")
        haxe_file = find_code_file(OIMO_PHYSICS_PATH, args.basename, ".hx")
    except FileNotFoundError as e:
        print(f"Could not find code files: {e}")
        return False

    go_code = load_file(go_file)
    haxe_code = load_file(haxe_file)

    client = genai.Client()

    prompt = f"""
This project is to port OimoPhysics a 3D physics engine written using Haxe, to Go.

You are an expert code reviewer.

Note that these are not issues:

* Go range statement directly used with integer `for i := range <integer>{{}}` or in the form `for range <integer> {{}}`
* Using `self` as the receiver name for all methods

Given the following origianl Haxe file and the Go port file, compare them and write a review that identifies and lists issues in the Go file grouped into:

- Errors (incorrect logic, missing parts, broken behavior): numbered list
- Warnings (possible mistakes, suspicious differences): numbered list
- Suggestions (cleanups, idiomatic improvements, anything else): numbered list

Return the result **as Markdown**, using this structure:

# Review Report

Haxe file: ModuleName.hx
Go file: ModuleName.go

## Errors

1. ...

## Warnings

1. ...

## Suggestions

1. ...

--- HAXE SOURCE ---
{haxe_code}

--- GO CONVERSION ---
{go_code}
"""

    print(f"Reviewing {os.path.basename(haxe_file)} and {os.path.basename(go_file)}..")

    response = client.models.generate_content(
        model="gemini-2.5-pro",
        contents=prompt,
        config=GenerateContentConfig(
            temperature=0,
        ),
    )

    # print(response.text)
    markdown_output = response.text
    write_output(REVIEW_FILENAME, markdown_output)

    print("---------------------------------------------")
    print_markdown(REVIEW_FILENAME)


if __name__ == "__main__":
    main()
