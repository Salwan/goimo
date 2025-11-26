#!/usr/bin/env python3
# LLM
# Usage: run and enter a Haxe visibility pattern (examples below)
# Outputs a suggested Go identifier form and brief notes.

import re

MAP = {
    'public': {
        'desc': 'Exported across packages',
        'suggest': lambda name: capitalize(name),
        'example': 'public var velocity -> Velocity'
    },
    'private': {
        'desc': 'Unexported (package-private)',
        'suggest': lambda name: uncapitalize(name),
        'example': 'private var mass -> mass'
    },
    'protected': {
        'desc': 'No direct Go equivalent — keep unexported and place types in same package, or export if cross-package needed',
        'suggest': lambda name: uncapitalize(name),
        'example': 'protected function update -> update or Update (if exported)'
    },
    'internal': {
        'desc': 'Package-level visibility in Go: use unexported identifiers',
        'suggest': lambda name: uncapitalize(name),
        'example': 'internal var counter -> counter'
    },
    'static': {
        'desc': 'Use package-level func/var or exported name if public',
        'suggest': lambda name: capitalize(name) if looks_public(name) else uncapitalize(name),
        'example': 'static function Create -> Create() or create() as package-level'
    },
    'get': {
        'desc': 'Use method getter or exported field; booleans use IsX',
        'suggest': lambda name: ('Is' + capitalize(name)) if name.startswith('is_') or name.startswith('has_') else capitalize(name),
        'example': 'get x -> X() or exported field X'
    },
    'set': {
        'desc': 'Use SetX method',
        'suggest': lambda name: 'Set' + capitalize(name),
        'example': 'set x -> SetX(v)'
    },
    'constructor': {
        'desc': 'Use NewType(...) constructor function',
        'suggest': lambda name: 'New' + capitalize(name),
        'example': 'new Body -> NewBody(...)'
    },
    'extern': {
        'desc': 'External/foreign binding — preserve original name and map types; expose as exported Go identifier if used across packages',
        'suggest': lambda name: capitalize(name) if name and not name.startswith('_') else uncapitalize(name.lstrip('_')),
        'example': 'extern function js_create -> JsCreate (exported wrapper) or js_create -> jsCreate (if kept internal)'
    }
}

def capitalize(s):
    s = s.strip()
    s = re.sub(r'[_\s]+', ' ', s).title().replace(' ', '')
    if not s:
        return s
    return s[0].upper() + s[1:]

def uncapitalize(s):
    s = s.strip()
    s = re.sub(r'[_\s]+', ' ', s).title().replace(' ', '')
    if not s:
        return s
    return s[0].lower() + s[1:]

def looks_public(name):
    # heuristic: explicit 'public' mention or leading uppercase in example input
    return bool(re.search(r'public', name, re.IGNORECASE))

def parse_input(inp):
    # Accept forms like: "public var foo", "private function _bar", "get x", "static function create"
    inp = inp.strip()
    parts = inp.split()
    if not parts:
        return None
    visibility = parts[0].lower()
    # pick last token as identifier if present
    identifier = parts[-1] if len(parts) > 1 else ''
    # strip punctuation
    identifier = re.sub(r'[():]', '', identifier)
    return visibility, identifier

def suggest(visibility, identifier):
    if visibility in MAP:
        entry = MAP[visibility]
        sugg = entry['suggest'](identifier)
        return {
            'haxe': visibility + (' ' + identifier if identifier else ''),
            'go': sugg,
            'note': entry['desc'],
            'example': entry.get('example', '')
        }
    # fallback rules
    if visibility in ('var', 'function', 'method'):
        # no explicit visibility: assume public if capitalized, else private
        if identifier and identifier[0].isupper():
            return {'haxe': visibility + ' ' + identifier, 'go': capitalize(identifier), 'note': 'Assume exported'}
        else:
            return {'haxe': visibility + ' ' + identifier, 'go': uncapitalize(identifier), 'note': 'Assume unexported'}
    return {'haxe': visibility + (' ' + identifier if identifier else ''), 'go': uncapitalize(identifier or visibility), 'note': 'Unknown visibility — default to unexported'}

def main():
    print("Haxe → Go naming helper (enter blank to exit). Examples: 'public var velocity', 'private function _mass', 'get x', 'constructor Body'")
    try:
        while True:
            inp = input('> ').strip()
            if not inp:
                break
            parsed = parse_input(inp)
            if not parsed:
                print("Couldn't parse input.")
                continue
            vis, ident = parsed
            out = suggest(vis, ident)
            print(f"Go identifier: {out['go']}")
            print(f"Note: {out['note']}")
            if out['example']:
                print(f"Example: {out['example']}")
            print()
    except (EOFError, KeyboardInterrupt):
        print("\nbye")

if __name__ == '__main__':
    main()

