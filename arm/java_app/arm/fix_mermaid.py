import re

file_path = r'c:\Users\DELL\Documents\NetBeans\BTL\diagrams_report.md'

with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace `\\n` with a placeholder
content = content.replace(r'\\n', '_ESCAPED_NEWLINE_')

# Replace `\n` with `<br>`
content = content.replace(r'\n', '<br>')

# Restore `\\n`
content = content.replace('_ESCAPED_NEWLINE_', r'\\n')

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Done")
