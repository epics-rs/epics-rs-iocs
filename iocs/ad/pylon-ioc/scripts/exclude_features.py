"""Drop records of excluded features from a makeDb.py template.

usage: exclude_features.py <template> <excluded_features.txt>
rewrites <template> in place and prints what it dropped
"""

import re
import sys

OUTPUT_RECORDS = {"ao", "bo", "mbbo", "longout", "int64out", "stringout"}
RECORD = re.compile(r'record\((\w+), "[^"]*"\) \{\n.*?\n\}\n\n', re.S)
FEATURE = re.compile(r'\)GC_[A-Z]_(\w+)"')


def rules(path):
    out = []
    for line in open(path):
        line = line.strip()
        if line and not line.startswith("#"):
            scope, pattern = line.split(None, 1)
            if scope not in ("all", "output"):
                sys.exit(f"{path}: unknown scope {scope!r}")
            out.append((scope, re.compile(pattern)))
    return out


def main():
    template, rules_path = sys.argv[1:3]
    excluded = rules(rules_path)
    text = open(template).read()
    dropped = {}

    def keep(m):
        feature = FEATURE.search(m.group(0)).group(1)
        for scope, pattern in excluded:
            if pattern.fullmatch(feature) and (scope == "all" or m.group(1) in OUTPUT_RECORDS):
                dropped.setdefault(feature, []).append(m.group(1))
                return ""
        return m.group(0)

    text = RECORD.sub(keep, text)
    open(template, "w").write(text)
    for feature in sorted(dropped):
        print(f"  dropped {feature}: {' '.join(dropped[feature])}")
    print(f"{len(dropped)} features trimmed, {text.count('record(')} records left")


if __name__ == "__main__":
    main()
