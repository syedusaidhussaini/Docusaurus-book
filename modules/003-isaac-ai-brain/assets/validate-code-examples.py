# Validation script for Module 3 code examples
# This script validates that the code examples in the Isaac Sim examples file are syntactically correct

import ast
import os
from pathlib import Path

def extract_python_code_from_file(file_path):
    """Extract only Python code from a file that may contain markdown or other text"""
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Split content by code block markers
    lines = content.split('\n')
    python_code = []
    in_code_block = False

    for line in lines:
        if line.strip().startswith('```python') or line.strip().startswith('## '):
            in_code_block = True
            if line.strip().startswith('```python'):
                continue  # Skip the marker line
        elif line.strip() == '```' and in_code_block:
            in_code_block = False
        elif in_code_block:
            python_code.append(line)

    return '\n'.join(python_code)

def validate_python_syntax(file_path):
    """Validate Python syntax in the given file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # For files that contain mixed content (like the Isaac examples), extract Python code
        if 'isaac-sim-examples.py' in file_path:
            python_code = extract_python_code_from_file(file_path)
        else:
            python_code = content

        # Parse the Python code to check for syntax errors
        ast.parse(python_code)
        print(f"[SUCCESS] {file_path} - Syntax is valid")
        return True
    except SyntaxError as e:
        print(f"[ERROR] {file_path} - Syntax error: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] {file_path} - Error: {e}")
        return False

def extract_python_code_blocks(markdown_file):
    """Extract Python code blocks from a markdown file"""
    with open(markdown_file, 'r', encoding='utf-8') as file:
        content = file.read()

    # Find code blocks with python/python3 syntax
    import re
    pattern = r'```python\n(.*?)\n```'
    matches = re.findall(pattern, content, re.DOTALL)

    return matches

def validate_code_examples():
    """Validate all code examples in Module 3"""
    print("Validating Module 3 code examples...")

    # Validate the main Isaac Sim examples file
    isaac_examples_path = "ui-docusaurus/modules/003-isaac-ai-brain/assets/code-examples/isaac-sim-examples.py"
    if os.path.exists(isaac_examples_path):
        success = validate_python_syntax(isaac_examples_path)
        if success:
            print("✓ Isaac Sim examples file is valid")
        else:
            print("✗ Isaac Sim examples file has errors")
    else:
        print(f"✗ File not found: {isaac_examples_path}")

    # Check for any Python code blocks in the module chapters
    chapter_dirs = [
        "ui-docusaurus/modules/003-isaac-ai-brain/chapter-7-isaac-sim-photorealistic-simulation/index.md",
        "ui-docusaurus/modules/003-isaac-ai-brain/chapter-8-synthetic-data-generation/index.md",
        "ui-docusaurus/modules/003-isaac-ai-brain/chapter-9-isaac-ros-vslam-nav2/index.md"
    ]

    for chapter_path in chapter_dirs:
        if os.path.exists(chapter_path):
            print(f"\nChecking code blocks in {chapter_path}:")
            code_blocks = extract_python_code_blocks(chapter_path)
            for i, code_block in enumerate(code_blocks):
                # Create a temporary file to validate the code block
                try:
                    ast.parse(code_block)
                    print(f"  ✓ Code block {i+1} - Valid syntax")
                except SyntaxError as e:
                    print(f"  ✗ Code block {i+1} - Syntax error: {e}")
        else:
            print(f"✗ Chapter file not found: {chapter_path}")

    print("\nValidation complete!")

if __name__ == "__main__":
    validate_code_examples()