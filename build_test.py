#!/usr/bin/env python3

import os
import re
import sys
from pathlib import Path

def check_header_guards(file_path):
    with open(file_path, 'r') as f:
        content = f.read()
    
    if not file_path.endswith('.h'):
        return True
    
    has_ifndef = '#ifndef' in content
    has_define = '#define' in content
    has_endif = '#endif' in content
    
    if not (has_ifndef and has_define and has_endif):
        print(f"Warning: Missing include guards in {file_path}")
        return False
    
    return True

def check_circular_includes():
    src_dir = Path('src')
    include_dir = Path('include')
    
    include_map = {}
    
    for h_file in include_dir.glob('*.h'):
        with open(h_file, 'r') as f:
            includes = re.findall(r'#include\s+"([^"]+)"', f.read())
        include_map[h_file.name] = includes
    
    for file_name, includes in include_map.items():
        for included in includes:
            if included in include_map:
                if file_name in include_map[included]:
                    print(f"Warning: Potential circular dependency: {file_name} <-> {included}")
                    return False
    
    return True

def check_missing_implementations():
    include_dir = Path('include')
    src_dir = Path('src')
    
    headers = list(include_dir.glob('*.h'))
    sources = list(src_dir.glob('*.cpp'))
    
    source_names = [s.stem for s in sources]
    
    for header in headers:
        if header.stem not in source_names and header.stem != 'FreeRTOSConfig':
            print(f"Warning: No implementation file for {header.name}")
    
    return True

def check_syntax_errors():
    src_dir = Path('src')
    include_dir = Path('include')
    
    for cpp_file in src_dir.glob('*.cpp'):
        with open(cpp_file, 'r') as f:
            content = f.read()
        
        if content.count('{') != content.count('}'):
            print(f"Error: Unmatched braces in {cpp_file.name}")
            return False
        
        if content.count('(') != content.count(')'):
            print(f"Error: Unmatched parentheses in {cpp_file.name}")
            return False
        
        lines = content.split('\n')
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped and not stripped.startswith('//') and not stripped.startswith('#'):
                if stripped.endswith('{') or stripped.endswith('}'):
                    continue
                if any(keyword in stripped for keyword in ['if', 'for', 'while', 'class', 'struct']):
                    continue
                if 'class ' in stripped or 'struct ' in stripped:
                    continue
                if not any(char in stripped for char in '{;'):
                    pass
    
    return True

def check_include_paths():
    src_dir = Path('src')
    include_dir = Path('include')
    
    for cpp_file in src_dir.glob('*.cpp'):
        with open(cpp_file, 'r') as f:
            content = f.read()
        
        local_includes = re.findall(r'#include\s+"([^"]+)"', content)
        
        for include in local_includes:
            if not (include_dir / include).exists():
                print(f"Error: Missing include file: {include} (referenced in {cpp_file.name})")
                return False
    
    return True

def main():
    print("Running Stewart Platform build tests...")
    print("=" * 50)
    
    project_dir = Path.cwd()
    
    os.chdir(project_dir)
    
    tests = [
        ("Include guards", check_header_guards),
        ("Circular dependencies", check_circular_includes),
        ("Missing implementations", check_missing_implementations),
        ("Syntax errors", check_syntax_errors),
        ("Include paths", check_include_paths),
    ]
    
    all_passed = True
    
    for test_name, test_func in tests:
        print(f"\n{test_name}...")
        try:
            result = test_func()
            if result:
                print(f"✓ {test_name} passed")
            else:
                print(f"✗ {test_name} failed")
                all_passed = False
        except Exception as e:
            print(f"✗ {test_name} error: {e}")
            all_passed = False
    
    print("\n" + "=" * 50)
    if all_passed:
        print("All tests passed!")
        return 0
    else:
        print("Some tests failed. Please review the errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
