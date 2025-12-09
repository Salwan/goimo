#!/usr/bin/env python3
# OpenRouter-driven code reviewer (partial-LLM)
# python reqs: pip3 install requests rich
# dnf reqs: dnf install glow

# Wishlist:
# [ ] add new arg (maybe: `-add BroadPhase,ArrayList`) to allow adding dependency modules to help with evaluation

import argparse
import json
import shutil
import shlex
import subprocess
import os
import sys
import requests

SETTINGS_FILENAME = os.path.expanduser("~/.OimoPhysics.reviewer")
REVIEW_FILENAME = "review.result.md"
OIMO_PHYSICS_PATH = os.path.expanduser("~/dev/OimoPhysics")

OPENROUTER_API_URL = "https://openrouter.ai/api/v1/chat/completions"
OPENROUTER_MODELS = [
    "tngtech/deepseek-r1t2-chimera:free",   # 670B R1+Coder
    "google/gemma-3-27b-it:free",           # 27B, worse than DS but may catch strays
    "qwen/qwen3-coder:free",                # 480B, not as good as DS/Gemma
    #"openai/gpt-oss-120b:free",             # 120B, limited length, maybe for free tier specifically
]
DEFAULT_MODEL=0

def configure(model_index):
    # This does not modify current environment, only python's os.environ map
    print("Configuring..")
    script_path = shlex.quote(SETTINGS_FILENAME)
    command = ['bash', '-c', f'source "{script_path}" >/dev/null 2>&1 && env']
    result = subprocess.run(
        command, capture_output=True, text=True, check=True
    )
    for line in result.stdout.splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            os.environ[key] = value
    print(f"Using model: {OPENROUTER_MODELS[model_index]}")

def load_file(path):
    with open(path, "r") as f:
        return f.read()

def write_output(path, content):
    with open(path, "w") as f:
        f.write(content)

def print_markdown(review_path):
    glow_path = shutil.which("glow")
    if glow_path is None:
        with open(review_path) as f:
            print(f.read())
        return
    try:
        subprocess.run([glow_path, review_path], check=True)
    except subprocess.CalledProcessError:
        with open(review_path) as f:
            print(f.read())

def find_code_file(search_path, base_name, extension):
    target_filename = base_name + extension
    for dirpath, dirnames, filenames in os.walk(search_path):
        if target_filename in filenames:
            return os.path.join(dirpath, target_filename)
    raise FileNotFoundError(f"File '{target_filename}' not found under '{search_path}'")

def print_models():
    print("Available models:")
    for i in range(len(OPENROUTER_MODELS)):
        print(f"  [{i}] {OPENROUTER_MODELS[i]}")
    print()

def call_openrouter(prompt, api_key, model_index):
    headers = {
        "Authorization": f"Bearer {api_key}",
        "HTTP-Referer": "http://localhost",     # required by OpenRouter
        "X-Title": "Code Reviewer",             # optional
        "Content-Type": "application/json",
    }

    payload = {
        "model": OPENROUTER_MODELS[model_index],
        "temperature": 0,
        "messages": [
            {
                "role": "user",
                "content": prompt
            }
        ]
    }

    resp = requests.post(OPENROUTER_API_URL, headers=headers, json=payload)
    if resp.status_code != 200:
        raise RuntimeError(f"OpenRouter API error {resp.status_code}: {resp.text}")

    data = resp.json()
    try:
        finish_reason = data["choices"][0]["finish_reason"]
        if finish_reason == "length":
            raise RuntimeError("Prompt+Response too long for model " + OPENROUTER_MODELS[model_index])
        d = data["choices"][0]["message"]["content"]
        if len(d) <= 1:
            raise RuntimeError("Returned response is empty.")
        else:
            return d
    except RuntimeError as e:
        raise e
    except Exception:
        raise RuntimeError("Unexpected OpenRouter response format: " + json.dumps(data, indent=2))

def main():
    parser = argparse.ArgumentParser(
        description="Review Go↔Haxe conversion using OpenRouter."
    )
    parser.add_argument(
        "basename",
        help="Object name to review, searches for both Go (in current path) and Haxe (in configured OimoPhysics path)"
    )
    parser.add_argument(
        "note",
        nargs='?',
        default="",
        help="Optional text to append to prompt to guide the review when needed, appended to: \"Note that: ...\""
    )
    parser.add_argument(
        "-m", "--model",
        default=DEFAULT_MODEL,
        help="Select model from preset list"
    )
    
    if len(sys.argv) == 1:
        parser.print_help()
        print()
        print_models()
        sys.exit(0)
    
    args = parser.parse_args()

    selected_model=int(args.model)
    note_that=""

    if len(args.note) > 0:
        note_that="Note that for this review: " + str(args.note)

    if selected_model < 0 or selected_model >= len(OPENROUTER_MODELS):
        print(f"Model index {selected_model} invalid.")
        print_models()
        return False

    try:
        configure(selected_model)
    except subprocess.CalledProcessError as e:
        print("Failed to execute environment script:")
        print(e.stderr)
        return False

    if "OPENROUTER_API_KEY" not in os.environ:
        print("OpenRouter API key not set: OPENROUTER_API_KEY")
        return False

    try:
        go_file = find_code_file(".", args.basename, ".go")
        haxe_file = find_code_file(OIMO_PHYSICS_PATH, args.basename, ".hx")
    except FileNotFoundError as e:
        print(f"Could not find code files: {e}")
        return False

    go_code = load_file(go_file)
    haxe_code = load_file(haxe_file)

    module_name = os.path.basename(haxe_file)

    prompt = f"""
This project is to port OimoPhysics, a 3D physics engine written in Haxe, to Go.

You are an expert code reviewer.

{note_that}

Note that these are not issues:
* Go range statement directly used with integer `for i := range <integer>{{}}` or in the form `for range <integer> {{}}`
* Using `self` as the receiver name for all methods

Given the following original Haxe file and the Go port file, compare them and write a review that identifies and lists issues in the Go file grouped into:

- Errors (incorrect logic, missing parts, broken behavior): numbered list
- Warnings (possible mistakes, suspicious differences): numbered list
- Suggestions (cleanups, idiomatic improvements, anything else): numbered list

Return the result as Markdown, using this structure:

# Review Report

Haxe file: {os.path.basename(haxe_file)}
Go file: {os.path.basename(go_file)}

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

    try:
        markdown_output = call_openrouter(
            prompt,
            api_key=os.environ["OPENROUTER_API_KEY"],
            model_index=selected_model
        )

        write_output(REVIEW_FILENAME, markdown_output)

        print("---------------------------------------------")
        print_markdown(REVIEW_FILENAME)
    except Exception as e:
        print("OpenRouter returned error: " + str(e))

if __name__ == "__main__":
    main()
