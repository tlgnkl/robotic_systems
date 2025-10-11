#!/usr/bin/env python3
"""
Скрипт для запуска всех модулей с деревьями
"""

import subprocess
import sys

def run_script(script_name):
    """Запускает Python скрипт и выводит результаты"""
    print(f"\n{'='*60}")
    print(f"Запуск: {script_name}")
    print('='*60)
    
    try:
        result = subprocess.run(
            [sys.executable, script_name],
            capture_output=True,
            text=True,
            check=True
        )
        print(result.stdout)
        if result.stderr:
            print("Предупреждения:", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Ошибка при запуске {script_name}:")
        print(e.stdout)
        print(e.stderr)
        return False

def main():
    scripts = [
        "basic_tree.py",
        "binary_tree.py",
        "binary_search_tree.py",
        "tree_traversal.py",
        "expression_tree.py",
        "file_system_tree.py",
        "decision_tree.py",
    ]
    
    print("Запуск всех модулей с деревьями...")
    
    results = {}
    for script in scripts:
        results[script] = run_script(script)
    
    # Итоговый отчет
    print(f"\n{'='*60}")
    print("ИТОГОВЫЙ ОТЧЕТ")
    print('='*60)
    
    success_count = sum(results.values())
    total_count = len(results)
    
    for script, success in results.items():
        status = "✅ OK" if success else "❌ FAIL"
        print(f"{status:8} - {script}")
    
    print(f"\nИтого: {success_count}/{total_count} успешно")
    
    return 0 if success_count == total_count else 1

if __name__ == "__main__":
    sys.exit(main())
