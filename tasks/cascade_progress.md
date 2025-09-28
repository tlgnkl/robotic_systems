# Progress Report

## Data Types Exercises

- **Exercise 1 – temperature_converter**
  - File: `tasks/task01_temperature_converter.py`
  - Summary: Converts temperature between Celsius, Fahrenheit, Kelvin with validation.
  - Tests: Automated via patched stdin; confirmed C→F→K, F→C conversions.

- **Exercise 2 – text_analyzer**
  - File: `tasks/task02_text_analyzer.py`
  - Summary: Reports character, word, sentence counts, longest word, letter case share, digits.
  - Tests: Mocked input sentence; validated presence of key metrics in output.

- **Exercise 3 – list_calculator**
  - File: `tasks/task03_list_calculator.py`
  - Summary: Calculates sum, average, squares, filters >30, sorts descending for preset list.
  - Tests: Captured stdout; checked required result sections.

- **Exercise 4 – financial_calculator**
  - File: `tasks/task04_financial_calculator.py`
  - Summary: Computes compound interest, compares with target, estimates doubling time.
  - Tests: Mocked stdin to cover main scenario; validated goal flag and doubling text.

- **Exercise 5 – advanced_password_checker**
  - File: `tasks/task05_advanced_password_checker.py`
  - Summary: Evaluates password strength across six criteria, capped 5-point score, gives tips.
  - Tests: Mocked three passwords (weak length, strong, common word) to confirm scoring and advice.

- **Exercise 6 – tax_calculator**
  - File: `tasks/task06_tax_calculator.py`
  - Summary: computes tax across progressive brackets, prints breakdown and net income.
  - Tests: Mocked incomes at 10k, 40k, 120k; verified no-tax case, bracket detail, top rate.

- **Exercise 7 – weather_advisor**
  - File: `tasks/task07_weather_advisor.py`
  - Summary: Detects season by month, suggests clothing by temperature, warns on extremes.
  - Tests: Mocked winter cold, summer heat, autumn mild scenarios to confirm advice and warnings.

- **Exercise 8 – math_examples_generator**
  - File: `tasks/task08_math_examples_generator.py`
  - Summary: Generates 10 arithmetic tasks with +/-/*/, checks answers, prints stats.
  - Tests: Fixed RNG seed with mocked answers; verified result summary output.

- **Exercise 9 – number_sequence_analyzer**
  - File: `tasks/task09_number_sequence_analyzer.py`
  - Summary: Collects numbers, reports stats, detects duplicates, sorts, renders ASCII graph.
  - Tests: Mocked sequence with repeat; asserted counts, max/min, duplicate list, graph presence.

- **Exercise 10 – bulls_and_cows**
  - File: `tasks/task10_bulls_and_cows.py`
  - Summary: Implements classic 4-digit Bulls & Cows with validation and 10-attempt limit.
  - Tests: Patched secret to `1234`; checked invalid guess handling and win scenario output.

- **Exercise 11 – student_management_system**
  - File: `tasks/task11_student_management_system.py`
  - Summary: Menu-driven CRUD for students (add/list/search/stats) with formatted output.
  - Tests: Simulated menu flow (add, list, search, stats, exit); asserted key messages.

- **Exercise 12 – report_generator**
  - File: `tasks/task12_report_generator.py`
  - Summary: Aggregates sales data to compute total revenue, top quantity, highest revenue, avg check.
  - Tests: Captured stdout; asserted presence of all required report lines.
